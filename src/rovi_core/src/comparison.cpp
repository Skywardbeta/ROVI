#include "rovi_core/comparison.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/cohort.hpp"
#include "rovi_core/engine.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/receipt.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

constexpr WorkerId kLane{1};
constexpr ModelClass kModel{1};

struct Named {
  std::string_view name;
  ComparisonPolicy policy;
};

constexpr std::array<Named, 8> kNames{{
    {.name = "linked_least_slack", .policy = ComparisonPolicy::kLinkedLeastSlack},
    {.name = "linked_earliest_deadline", .policy = ComparisonPolicy::kLinkedEarliestDeadline},
    {.name = "linked_first_in_first_out", .policy = ComparisonPolicy::kLinkedFirstInFirstOut},
    {.name = "independent_earliest_deadline",
     .policy = ComparisonPolicy::kIndependentEarliestDeadline},
    {.name = "independent_equal_share", .policy = ComparisonPolicy::kIndependentEqualShare},
    {.name = "least_slack", .policy = ComparisonPolicy::kLinkedLeastSlack},
    {.name = "earliest_deadline", .policy = ComparisonPolicy::kLinkedEarliestDeadline},
    {.name = "first_in_first_out", .policy = ComparisonPolicy::kLinkedFirstInFirstOut},
}};

std::optional<Policy> linked_order(ComparisonPolicy policy) {
  switch (policy) {
    case ComparisonPolicy::kLinkedLeastSlack:
      return Policy::kLeastSlack;
    case ComparisonPolicy::kLinkedEarliestDeadline:
      return Policy::kEarliestDeadline;
    case ComparisonPolicy::kLinkedFirstInFirstOut:
      return Policy::kFirstInFirstOut;
    case ComparisonPolicy::kIndependentEarliestDeadline:
    case ComparisonPolicy::kIndependentEqualShare:
      return std::nullopt;
  }
  return std::nullopt;
}

// Everything a policy may see or do at one decision time. The requests released by then are a
// prefix of the cohort in release order, and touching a later one throws. Policies hold no engine,
// so they can neither read ahead nor grant link service themselves.
class View {
 public:
  View(Engine& engine, std::size_t released, const std::vector<std::size_t>& active)
      : engine_{&engine}, released_{released}, active_{&active} {}

  [[nodiscard]] std::size_t released() const { return released_; }
  // Released requests not yet received or dropped, which the independent stages work through.
  [[nodiscard]] const std::vector<std::size_t>& active() const { return *active_; }
  [[nodiscard]] const Request& request(std::size_t index) const {
    return engine_->request(checked(index));
  }
  [[nodiscard]] const Progress& progress(std::size_t index) const {
    return engine_->progress(checked(index));
  }
  // Whether the robot has actually sent the observation by the time, which the planner never sees.
  [[nodiscard]] bool sent_by(std::size_t index, TimePoint time) const {
    const Request& offered = request(index);
    const std::optional<TimePoint> sent = checked_add(offered.release, offered.send_delay);
    return sent && *sent <= time;
  }
  [[nodiscard]] TimePoint slot_start(std::int64_t slot) const { return engine_->slot_start(slot); }
  [[nodiscard]] Bytes unused(Link link, std::int64_t slot) const {
    return engine_->unused(link, slot);
  }
  [[nodiscard]] std::optional<Bytes> unused_by(Link link, RobotId robot, std::int64_t slot) const {
    return engine_->unused_by(link, robot, slot);
  }
  [[nodiscard]] TimePoint lane_free() const { return engine_->lane_free(); }
  [[nodiscard]] bool start_compute(std::size_t index, TimePoint at) {
    return engine_->start_compute(checked(index), at);
  }

 private:
  [[nodiscard]] std::size_t checked(std::size_t index) const {
    if (index >= released_) {
      throw std::logic_error{"a policy touched a request before its release"};
    }
    return index;
  }

  Engine* engine_;
  std::size_t released_;
  const std::vector<std::size_t>* active_;
};

class SlotBudget {
 public:
  SlotBudget(const View& view, Link link, std::int64_t slot)
      : view_{&view}, link_{link}, slot_{slot}, left_{view.unused(link, slot).value()} {}

  [[nodiscard]] std::int64_t left() const { return left_; }

  std::int64_t take(std::size_t index, Bytes wanted) {
    std::int64_t granted = std::clamp<std::int64_t>(wanted.value(), 0, left_);
    std::int64_t* robot_left = robot_budget(index);
    if (robot_left != nullptr) {
      granted = std::min(granted, *robot_left);
      *robot_left -= granted;
    }
    left_ -= granted;
    return granted;
  }

 private:
  std::int64_t* robot_budget(std::size_t index) {
    const RobotId robot = view_->request(index).robot;
    const std::optional<Bytes> cap = view_->unused_by(link_, robot, slot_);
    if (!cap) {
      return nullptr;
    }
    return &robot_left_.try_emplace(robot, cap->value()).first->second;
  }

  const View* view_;
  Link link_;
  std::int64_t slot_;
  std::int64_t left_;
  std::map<RobotId, std::int64_t> robot_left_;
};

std::optional<TimePoint> ready_for_lane(const Progress& state) {
  return state.compute_start || state.dropped ? std::nullopt : state.ready;
}

bool earlier_deadline(const View& view, std::size_t lhs, std::size_t rhs) {
  const Request& left = view.request(lhs);
  const Request& right = view.request(rhs);
  return std::tie(left.deadline, left.release, left.id) <
         std::tie(right.deadline, right.release, right.id);
}

std::vector<std::size_t> waiting_on(const View& view, Link link, std::int64_t slot,
                                    const std::vector<std::size_t>& candidates) {
  const TimePoint start = view.slot_start(slot);
  std::vector<std::size_t> waiting;
  for (const std::size_t index : candidates) {
    const Progress& state = view.progress(index);
    const Bytes left = link == Link::kUplink ? state.upload_left : state.response_left;
    const bool exists = link == Link::kUplink ? view.sent_by(index, start)
                                              : state.produced && *state.produced <= start;
    if (!state.dropped && exists && left > Bytes{0}) {
      waiting.push_back(index);
    }
  }
  std::ranges::sort(waiting, [&view](std::size_t lhs, std::size_t rhs) {
    return earlier_deadline(view, lhs, rhs);
  });
  return waiting;
}

std::int64_t need_on(const View& view, Link link, std::size_t index) {
  const Progress& state = view.progress(index);
  return (link == Link::kUplink ? state.upload_left : state.response_left).value();
}

// Splits capacity evenly among the needs, giving any request that needs less than its share
// exactly its need; leftover single bytes go to the earliest entries.
std::vector<std::int64_t> equal_shares(const std::vector<std::int64_t>& needs,
                                       std::int64_t capacity) {
  std::vector<std::int64_t> shares(needs.size(), 0);
  std::vector<std::size_t> open(needs.size());
  for (std::size_t index = 0; index < open.size(); ++index) {
    open.at(index) = index;
  }
  while (!open.empty() && capacity > 0) {
    const std::int64_t share = capacity / std::ssize(open);
    std::vector<std::size_t> unmet;
    for (const std::size_t index : open) {
      const std::int64_t wanted = needs.at(index) - shares.at(index);
      const std::int64_t given = std::min(wanted, share);
      shares.at(index) += given;
      capacity -= given;
      if (given < wanted) {
        unmet.push_back(index);
      }
    }
    if (unmet.size() == open.size()) {
      for (std::size_t index = 0; index < unmet.size() && capacity > 0; ++index, --capacity) {
        ++shares.at(unmet.at(index));
      }
      break;
    }
    open = std::move(unmet);
  }
  return shares;
}

// Each stage ranks only work that already exists in front of it, by original deadline, and never
// plans or reserves a later stage. Equal share splits the link evenly before robot caps apply.
class IndependentStages {
 public:
  explicit IndependentStages(bool equal_share) : equal_share_{equal_share} {}

  [[nodiscard]] static bool compute(View& view, TimePoint until) {
    while (true) {
      std::optional<TimePoint> earliest;
      for (const std::size_t index : view.active()) {
        const Progress& state = view.progress(index);
        if (state.ready && !state.compute_start && !state.dropped &&
            (!earliest || *state.ready < *earliest)) {
          earliest = state.ready;
        }
      }
      if (!earliest) {
        return true;
      }
      const TimePoint at = std::max(view.lane_free(), *earliest);
      if (at >= until) {
        return true;
      }
      std::optional<std::size_t> chosen;
      for (const std::size_t index : view.active()) {
        const Progress& state = view.progress(index);
        if (state.ready && *state.ready <= at && !state.compute_start && !state.dropped &&
            (!chosen || earlier_deadline(view, index, *chosen))) {
          chosen = index;
        }
      }
      if (!chosen || !view.start_compute(*chosen, at)) {
        return false;
      }
    }
  }

  [[nodiscard]] std::vector<Grant> grants(const View& view, Link link, std::int64_t slot) const {
    const std::vector<std::size_t> waiting = waiting_on(view, link, slot, view.active());
    std::vector<std::int64_t> needs;
    needs.reserve(waiting.size());
    std::ranges::transform(waiting, std::back_inserter(needs),
                           [&view, link](std::size_t index) { return need_on(view, link, index); });
    SlotBudget budget{view, link, slot};
    const std::vector<std::int64_t> wanted =
        equal_share_ ? equal_shares(needs, budget.left()) : needs;
    std::vector<Grant> grants;
    for (std::size_t position = 0; position < waiting.size(); ++position) {
      const std::int64_t bytes = budget.take(waiting.at(position), Bytes{wanted.at(position)});
      if (bytes > 0) {
        grants.push_back(Grant{.request = waiting.at(position), .bytes = Bytes{bytes}});
      }
    }
    return grants;
  }

 private:
  bool equal_share_;
};

// Admits whole chains with the chain planner against cloned predicted calendars, then executes
// each reservation exactly. Strict, a reservation the data misses is lost and its request gets
// nothing more. With best effort, such a request, or one the planner rejects, is served from what
// the commitments leave unused, and its compute must fit a lane gap no reservation protects.
class LinkedPlans {
 public:
  LinkedPlans(Policy order, ResourceSnapshot predicted, std::size_t size,
              const RecoveryRules& rules)
      : order_{order},
        predicted_{std::move(predicted)},
        best_effort_{rules.best_effort},
        compute_bound_{rules.compute_bound},
        plans_(size),
        commitments_(size, Commitment::kUndecided),
        missed_(size, false),
        recovering_(size, false),
        unreserved_compute_(size, false) {}

  void decide(const View& view, TimePoint now) {
    for (; seen_ < view.released(); ++seen_) {
      undecided_.push_back(seen_);
    }
    if (undecided_.empty()) {
      return;
    }
    std::vector<Job> pending;
    std::map<JobId, std::size_t> index_of;
    for (const std::size_t index : undecided_) {
      pending.push_back(job_of(view.request(index), index));
      index_of.emplace(view.request(index).id, index);
    }
    ScheduleResult result = schedule(predicted_, pending, {}, now, order_);
    predicted_ = std::move(result.snapshot);
    for (const JobDecision& decision : result.decisions) {
      const std::size_t index = index_of.at(decision.job);
      if (decision.decision.status == DecisionStatus::kAdmitted && decision.reservation) {
        admit(index, *decision.reservation);
      } else if (decision.decision.status == DecisionStatus::kRejected) {
        commitments_.at(index) = Commitment::kRejected;
        recover(index);
      }
    }
    std::erase_if(undecided_, [this](std::size_t index) {
      return commitments_.at(index) != Commitment::kUndecided;
    });
  }

  [[nodiscard]] bool compute(View& view, std::int64_t slot) {
    const TimePoint until = view.slot_start(slot + 1);
    for (auto entry = compute_order_.begin(); entry != compute_order_.end();) {
      const auto [planned, index] = *entry;
      const Progress& state = view.progress(index);
      const std::optional<TimePoint> at =
          state.ready ? std::optional{std::max({planned, *state.ready, view.lane_free()})}
                      : std::nullopt;
      if (!at || *at >= until) {
        ++entry;
        continue;
      }
      if (!view.start_compute(index, *at)) {
        return false;
      }
      entry = compute_order_.erase(entry);
    }
    return compute_best_effort(view, slot);
  }

  // A reservation whose data is missing when its slice falls due is lost. Compute started inside
  // the slot ends after its boundary, so this is settled before the slot's compute.
  void expire_leases(const View& view, std::int64_t slot) {
    const TimePoint start = view.slot_start(slot);
    for (const Link link : {Link::kUplink, Link::kDownlink}) {
      Slices& planned = link == Link::kUplink ? uplink_ : downlink_;
      const auto due = planned.find(slot);
      if (due == planned.end()) {
        continue;
      }
      std::vector<std::pair<std::size_t, Bytes>> kept;
      for (const auto& slice : due->second) {
        const Progress& state = view.progress(slice.first);
        const bool exists = link == Link::kUplink ? view.sent_by(slice.first, start)
                                                  : state.produced && *state.produced <= start;
        if (missed_.at(slice.first) || !exists) {
          missed_.at(slice.first) = true;
          recover(slice.first);
        } else {
          kept.push_back(slice);
        }
      }
      due->second = std::move(kept);
    }
  }

  [[nodiscard]] std::vector<Grant> committed_grants(Link link, std::int64_t slot) {
    std::vector<Grant> grants;
    std::ranges::transform(slices(link, slot), std::back_inserter(grants), [](const auto& slice) {
      return Grant{.request = slice.first, .bytes = slice.second};
    });
    return grants;
  }

  [[nodiscard]] std::vector<Grant> best_effort_grants(const View& view, Link link,
                                                      std::int64_t slot) const {
    SlotBudget budget{view, link, slot};
    std::vector<Grant> grants;
    for (const std::size_t index : waiting_on(view, link, slot, recovering())) {
      const std::int64_t bytes = budget.take(index, Bytes{need_on(view, link, index)});
      if (bytes > 0) {
        grants.push_back(Grant{.request = index, .bytes = Bytes{bytes}});
      }
    }
    return grants;
  }

  [[nodiscard]] std::vector<std::size_t> take_entered() { return std::exchange(entered_, {}); }

  [[nodiscard]] Commitment commitment(std::size_t index) const { return commitments_.at(index); }
  [[nodiscard]] const std::optional<ChainReservation>& plan(std::size_t index) const {
    return plans_.at(index);
  }
  [[nodiscard]] bool missed(std::size_t index) const { return missed_.at(index); }
  [[nodiscard]] bool recovering(std::size_t index) const { return recovering_.at(index); }
  [[nodiscard]] bool unreserved_compute(std::size_t index) const {
    return unreserved_compute_.at(index);
  }

 private:
  using Slices = std::map<std::int64_t, std::vector<std::pair<std::size_t, Bytes>>>;

  static Job job_of(const Request& request, std::size_t index) {
    return Job{.id = request.id,
               .release_time = request.release,
               .observation_time = request.release,
               .buffer_empty_time = request.deadline,
               .safety_margin = Duration{},
               .task_deadline = std::nullopt,
               .maximum_result_age = std::nullopt,
               .upload_bytes = request.upload,
               .action_chunk_bytes = request.response,
               .model = kModel,
               .eligible_workers = {kLane},
               .robot = request.robot,
               .sequence = index};
  }

  void admit(std::size_t index, const ChainReservation& chain) {
    plans_.at(index) = chain;
    commitments_.at(index) = Commitment::kAdmitted;
    compute_order_.emplace(chain.compute.start, index);
    for (const ServiceSlice& slice : chain.upload.slices) {
      uplink_[slice.slot.value()].emplace_back(index, slice.bytes);
    }
    for (const ServiceSlice& slice : chain.download.slices) {
      downlink_[slice.slot.value()].emplace_back(index, slice.bytes);
    }
  }

  // A request served best effort keeps no reservation: its remaining slices and planned compute
  // are released, though the predicted calendars keep them, so no new commitment reuses them.
  void recover(std::size_t index) {
    if (!best_effort_ || recovering_.at(index)) {
      return;
    }
    recovering_.at(index) = true;
    recovering_list_.push_back(index);
    entered_.push_back(index);
    std::erase_if(compute_order_, [index](const auto& entry) { return entry.second == index; });
  }

  [[nodiscard]] const std::vector<std::size_t>& recovering() const { return recovering_list_; }

  // Fits the declared bound into the first lane gap no reservation protects, from the time the lane
  // and some input are both available, then starts the earliest-deadline work ready by the gap and
  // charges the gap to the predicted lane before any new commitment. Without a bound it waits.
  [[nodiscard]] bool compute_best_effort(View& view, std::int64_t slot) {
    if (!compute_bound_) {
      return true;
    }
    const TimePoint now = view.slot_start(slot);
    const TimePoint until = view.slot_start(slot + 1);
    IntervalCalendar& lane = predicted_.workers.front().lane;
    while (true) {
      std::optional<TimePoint> earliest;
      for (const std::size_t index : recovering()) {
        const std::optional<TimePoint> ready = ready_for_lane(view.progress(index));
        if (ready && (!earliest || *ready < *earliest)) {
          earliest = ready;
        }
      }
      if (!earliest) {
        return true;
      }
      const std::optional<Interval> gap =
          lane.earliest_fit(std::max({now, view.lane_free(), *earliest}), *compute_bound_);
      if (!gap || gap->start >= until) {
        return true;
      }
      std::optional<std::size_t> chosen;
      for (const std::size_t index : recovering()) {
        const std::optional<TimePoint> ready = ready_for_lane(view.progress(index));
        if (ready && *ready <= gap->start && (!chosen || earlier_deadline(view, index, *chosen))) {
          chosen = index;
        }
      }
      std::optional<IntervalCalendar> charged = lane.reserve(*gap);
      if (!chosen || !charged || !view.start_compute(*chosen, gap->start)) {
        return false;
      }
      lane = std::move(*charged);
      unreserved_compute_.at(*chosen) = true;
    }
  }

  std::vector<std::pair<std::size_t, Bytes>> slices(Link link, std::int64_t slot) {
    Slices& planned = link == Link::kUplink ? uplink_ : downlink_;
    const auto found = planned.find(slot);
    if (found == planned.end()) {
      return {};
    }
    std::vector<std::pair<std::size_t, Bytes>> due = std::move(found->second);
    planned.erase(found);
    return due;
  }

  Policy order_;
  ResourceSnapshot predicted_;
  bool best_effort_;
  std::optional<Duration> compute_bound_;
  std::vector<std::optional<ChainReservation>> plans_;
  std::vector<Commitment> commitments_;
  std::vector<bool> missed_;
  std::vector<bool> recovering_;
  std::vector<bool> unreserved_compute_;
  std::vector<std::size_t> recovering_list_;
  std::vector<std::size_t> entered_;
  std::size_t seen_{0};
  std::vector<std::size_t> undecided_;
  std::multimap<TimePoint, std::size_t> compute_order_;
  Slices uplink_;
  Slices downlink_;
};

std::optional<CapacityCalendar> calendar_of(const Environment& environment,
                                            const std::vector<ServiceSegment>& profile) {
  return build_service_calendar(TimePoint{}, environment.slot_length, environment.slot_count,
                                profile);
}

std::optional<CapacityCalendar> cap_of(const Environment& environment,
                                       const std::optional<Bytes>& cap) {
  if (!cap) {
    return std::nullopt;
  }
  return CapacityCalendar::create(
      TimePoint{}, environment.slot_length,
      std::vector<Bytes>(static_cast<std::size_t>(environment.slot_count), *cap));
}

// All robots' cap calendars together get no more slots than the engine allows one link.
constexpr std::int64_t kMaximumRobotSlots = 10'000'000;

std::optional<std::vector<EndpointBudget>> endpoint_budgets(const Environment& environment,
                                                            const std::vector<Request>& cohort) {
  std::vector<EndpointBudget> endpoints;
  if (!environment.endpoint_uplink && !environment.endpoint_downlink) {
    return endpoints;
  }
  std::set<RobotId> robots;
  std::ranges::transform(cohort, std::inserter(robots, robots.end()), &Request::robot);
  if (environment.slot_count <= 0 ||
      std::ssize(robots) > kMaximumRobotSlots / environment.slot_count) {
    return std::nullopt;
  }
  const std::optional<CapacityCalendar> uplink = cap_of(environment, environment.endpoint_uplink);
  const std::optional<CapacityCalendar> downlink =
      cap_of(environment, environment.endpoint_downlink);
  if (uplink.has_value() != environment.endpoint_uplink.has_value() ||
      downlink.has_value() != environment.endpoint_downlink.has_value()) {
    return std::nullopt;
  }
  endpoints.reserve(robots.size());
  std::ranges::transform(
      robots, std::back_inserter(endpoints), [&uplink, &downlink](RobotId robot) {
        return EndpointBudget{.robot = robot, .uplink = uplink, .downlink = downlink};
      });
  return endpoints;
}

std::optional<ResourceSnapshot> predicted_resources(const Environment& environment,
                                                    const std::vector<Request>& cohort) {
  std::optional<CapacityCalendar> uplink = calendar_of(environment, environment.uplink);
  std::optional<CapacityCalendar> downlink = calendar_of(environment, environment.downlink);
  std::optional<CapacityCalendar> medium =
      environment.medium ? calendar_of(environment, *environment.medium) : std::nullopt;
  if (!uplink || !downlink || (environment.medium && !medium)) {
    return std::nullopt;
  }
  std::optional<std::vector<EndpointBudget>> endpoints = endpoint_budgets(environment, cohort);
  if (!endpoints) {
    return std::nullopt;
  }
  return ResourceSnapshot{
      .uplink = std::move(*uplink),
      .downlink = std::move(*downlink),
      .workers = {Worker{.id = kLane,
                         .profiles = {{.model = kModel, .duration = environment.compute_planned}},
                         .lane = IntervalCalendar{}}},
      .shared_medium = std::move(medium),
      .uplink_delay = environment.uplink_delay,
      .downlink_delay = environment.downlink_delay,
      .dispatch_lead = Duration{},
      .source = ServiceSource::kLogical,
      .endpoints = std::move(*endpoints)};
}

// Applies the declared cancellation rules to the work served without a deadline promise.
class Cancellation {
 public:
  Cancellation(const RecoveryRules& rules, std::size_t size) : rules_{rules}, dropped_(size) {}

  void enter(Engine& engine, std::size_t index, TimePoint now) {
    if (const std::optional<DropReason> reason = due(engine.request(index), index, now)) {
      drop(engine, index, now, *reason);
    } else if (rules_.backlog && pool_.size() >= *rules_.backlog) {
      drop(engine, index, now, DropReason::kBacklogFull);
    } else {
      pool_.push_back(index);
    }
  }

  void released(std::size_t index, RobotId robot) { newest_[robot] = index; }

  void apply(Engine& engine, TimePoint now) {
    for (const std::size_t index : pool_) {
      if (const std::optional<DropReason> reason = due(engine.request(index), index, now)) {
        drop(engine, index, now, *reason);
      }
    }
    settle(engine);
  }

  void settle(const Engine& engine) {
    std::erase_if(pool_, [&engine](std::size_t index) {
      const Progress& state = engine.progress(index);
      return state.receipt || state.dropped;
    });
  }

  [[nodiscard]] const std::optional<DropReason>& dropped(std::size_t index) const {
    return dropped_.at(index);
  }

 private:
  [[nodiscard]] std::optional<DropReason> due(const Request& offered, std::size_t index,
                                              TimePoint now) const {
    const std::optional<TimePoint> expiry =
        rules_.maximum_age ? checked_add(offered.release, *rules_.maximum_age) : std::nullopt;
    if (expiry && now > *expiry) {
      return DropReason::kExpired;
    }
    if (rules_.supersede && newest_.at(offered.robot) != index) {
      return DropReason::kSuperseded;
    }
    return std::nullopt;
  }

  void drop(Engine& engine, std::size_t index, TimePoint now, DropReason reason) {
    if (engine.drop(index, now)) {
      dropped_.at(index) = reason;
    }
  }

  RecoveryRules rules_;
  std::vector<std::size_t> pool_;
  std::map<RobotId, std::size_t> newest_;
  std::vector<std::optional<DropReason>> dropped_;
};

constexpr std::array kLinks{Link::kUplink, Link::kDownlink};

// Work that starts the slot without a reservation comes under the cancellation rules before any
// compute or service: every new independent release, and each linked rejection or lost lease.
void enter_unreserved(Engine& engine, std::int64_t slot, const View& view,
                      std::optional<LinkedPlans>& linked, Cancellation& cancellation,
                      std::size_t first_new) {
  const TimePoint now = engine.slot_start(slot);
  if (!linked) {
    for (std::size_t index = first_new; index < view.released(); ++index) {
      cancellation.enter(engine, index, now);
    }
    return;
  }
  linked->decide(view, now);
  linked->expire_leases(view, slot);
  for (const std::size_t index : linked->take_entered()) {
    cancellation.enter(engine, index, now);
  }
}

// Every reserved slice on both links is served before unreserved work on either, so recovery
// never takes the medium or robot cap that a reservation needs.
bool serve_links(Engine& engine, const View& view, std::optional<LinkedPlans>& linked,
                 const IndependentStages& independent, std::int64_t slot) {
  if (!linked) {
    return std::ranges::all_of(kLinks, [&](Link link) {
      return engine.serve(link, slot, independent.grants(view, link, slot));
    });
  }
  const auto reserved = [&](Link link) {
    return engine.serve(link, slot, linked->committed_grants(link, slot));
  };
  const auto unreserved = [&](Link link) {
    return engine.serve(link, slot, linked->best_effort_grants(view, link, slot));
  };
  return std::ranges::all_of(kLinks, reserved) && std::ranges::all_of(kLinks, unreserved);
}

RunResult result_of(ComparisonPolicy policy, const RecoveryRules& recovery, const Engine& engine,
                    const std::optional<LinkedPlans>& linked, const Cancellation& cancellation) {
  RunResult result{.policy = policy,
                   .cohort = {},
                   .outcomes = {},
                   .events = engine.events(),
                   .horizon = engine.horizon(),
                   .digest = engine.digest(),
                   .recovery = recovery};
  for (std::size_t index = 0; index < engine.size(); ++index) {
    const Request& request = engine.request(index);
    const Progress& progress = engine.progress(index);
    result.cohort.push_back(request);
    result.outcomes.push_back(RequestOutcome{
        .outcome = classify_receipt(request.deadline, progress.receipt, engine.horizon()),
        .commitment = linked ? linked->commitment(index) : Commitment::kNone,
        .plan = linked ? linked->plan(index) : std::nullopt,
        .lease_missed = linked && linked->missed(index),
        .progress = progress,
        .best_effort = linked && linked->recovering(index),
        .unreserved_compute = linked && linked->unreserved_compute(index),
        .dropped = cancellation.dropped(index)});
  }
  return result;
}

}  // namespace

Capabilities capabilities_of(ComparisonPolicy policy) {
  const bool linked = linked_order(policy).has_value();
  return Capabilities{
      .cross_stage_planning = linked, .atomic_admission = linked, .capacity_lookahead = linked};
}

std::string_view name_of(ComparisonPolicy policy) {
  const auto* const named = std::ranges::find(kNames, policy, &Named::policy);
  return named == kNames.end() ? std::string_view{} : named->name;
}

std::optional<ComparisonPolicy> policy_named(std::string_view name) {
  const auto* const named = std::ranges::find(kNames, name, &Named::name);
  return named == kNames.end() ? std::nullopt : std::optional{named->policy};
}

std::optional<RunResult> run_comparison(ComparisonPolicy policy, const Environment& environment,
                                        const std::vector<Request>& cohort,
                                        const RecoveryRules& recovery) {
  std::optional<Engine> engine = Engine::create(environment, cohort);
  const bool bound_conservative =
      !recovery.compute_bound || *recovery.compute_bound >= environment.compute_planned;
  if (!engine || !bound_conservative || !std::ranges::is_sorted(cohort, {}, &Request::release)) {
    return std::nullopt;
  }
  const std::optional<ResourceSnapshot> predicted = predicted_resources(environment, cohort);
  if (!predicted) {
    return std::nullopt;
  }
  std::optional<LinkedPlans> linked;
  if (const std::optional<Policy> order = linked_order(policy)) {
    linked.emplace(*order, *predicted, cohort.size(), recovery);
  }
  const IndependentStages independent{policy == ComparisonPolicy::kIndependentEqualShare};
  Cancellation cancellation{recovery, cohort.size()};
  std::size_t released = 0;
  std::vector<std::size_t> active;
  for (std::int64_t slot = 0; slot < environment.slot_count; ++slot) {
    const TimePoint now = engine->slot_start(slot);
    const std::size_t first_new = released;
    for (; released < cohort.size() && cohort.at(released).release <= now; ++released) {
      cancellation.released(released, cohort.at(released).robot);
      if (!linked) {
        active.push_back(released);
      }
    }
    cancellation.apply(*engine, now);
    View view{*engine, released, active};
    enter_unreserved(*engine, slot, view, linked, cancellation, first_new);
    const TimePoint until = engine->slot_start(slot + 1);
    const bool computed =
        linked ? linked->compute(view, slot) : IndependentStages::compute(view, until);
    if (!computed || !serve_links(*engine, view, linked, independent, slot)) {
      return std::nullopt;
    }
    cancellation.settle(*engine);
    std::erase_if(active, [&engine](std::size_t index) {
      const Progress& state = engine->progress(index);
      return state.receipt || state.dropped;
    });
  }
  return result_of(policy, recovery, *engine, linked, cancellation);
}

}  // namespace rovi::core
