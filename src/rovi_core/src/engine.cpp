#include "rovi_core/engine.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/cohort.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

// Bounds the per-slot ledgers to 80 MB each.
constexpr std::int64_t kMaximumSlots = 10'000'000;

bool valid(const Environment& environment) {
  const auto non_negative = [](const std::optional<Bytes>& cap) {
    return !cap || *cap >= Bytes{0};
  };
  return environment.slot_length > Duration{} && environment.slot_count > 0 &&
         environment.slot_count <= kMaximumSlots && environment.uplink_delay >= Duration{} &&
         environment.downlink_delay >= Duration{} && environment.compute_planned > Duration{} &&
         environment.compute_actual > Duration{} && non_negative(environment.endpoint_uplink) &&
         non_negative(environment.endpoint_downlink);
}

bool valid(const std::vector<Request>& cohort) {
  if (cohort.empty() || std::ranges::any_of(cohort, [](const Request& request) {
        return request.upload <= Bytes{0} || request.response <= Bytes{0} ||
               request.release < TimePoint{} || request.send_delay < Duration{} ||
               !checked_add(request.release, request.send_delay);
      })) {
    return false;
  }
  std::vector<JobId> ids;
  ids.reserve(cohort.size());
  std::ranges::transform(cohort, std::back_inserter(ids), &Request::id);
  std::ranges::sort(ids);
  return std::ranges::adjacent_find(ids) == ids.end();
}

// Every stage time the engine can produce stays representable: compute starts before the horizon
// and transfers complete by it.
bool representable(const Environment& environment, TimePoint horizon) {
  return checked_add(horizon, environment.uplink_delay) &&
         checked_add(horizon, environment.downlink_delay) &&
         checked_add(horizon, environment.compute_actual);
}

}  // namespace

std::optional<Engine> Engine::create(Environment environment, std::vector<Request> cohort) {
  if (!valid(environment) || !valid(cohort)) {
    return std::nullopt;
  }
  const auto calendar = [&environment](const std::vector<ServiceSegment>& profile) {
    return build_service_calendar(TimePoint{}, environment.slot_length, environment.slot_count,
                                  profile);
  };
  std::optional<CapacityCalendar> uplink = calendar(environment.uplink);
  std::optional<CapacityCalendar> downlink = calendar(environment.downlink);
  std::optional<CapacityCalendar> medium =
      environment.medium ? calendar(*environment.medium) : std::nullopt;
  if (!uplink || !downlink || (environment.medium && !medium) ||
      !representable(environment, uplink->horizon_end())) {
    return std::nullopt;
  }
  return Engine{std::move(environment), std::move(cohort), std::move(*uplink), std::move(*downlink),
                std::move(medium)};
}

Engine::Engine(Environment environment, std::vector<Request> cohort, CapacityCalendar uplink,
               CapacityCalendar downlink, std::optional<CapacityCalendar> medium)
    : environment_{std::move(environment)},
      cohort_{std::move(cohort)},
      uplink_{std::move(uplink)},
      downlink_{std::move(downlink)},
      medium_{std::move(medium)} {
  progress_.reserve(cohort_.size());
  std::ranges::transform(cohort_, std::back_inserter(progress_), [](const Request& offered) {
    return Progress{.upload_left = offered.upload,
                    .response_left = offered.response,
                    .ready = std::nullopt,
                    .compute_start = std::nullopt,
                    .produced = std::nullopt,
                    .receipt = std::nullopt,
                    .dropped = std::nullopt};
  });
}

TimePoint Engine::slot_start(std::int64_t slot) const {
  return TimePoint::from_ns(std::clamp<std::int64_t>(slot, 0, environment_.slot_count) *
                            environment_.slot_length.ns());
}

TimePoint Engine::horizon() const { return uplink_.horizon_end(); }

Bytes Engine::capacity(Link link, std::int64_t slot) const {
  if (slot < 0 || slot >= environment_.slot_count) {
    return Bytes{0};
  }
  return (link == Link::kUplink ? uplink_ : downlink_).capacity(SlotIndex{slot});
}

Bytes Engine::unused(Link link, std::int64_t slot) const {
  if (slot < current_slot_ || slot >= environment_.slot_count) {
    return Bytes{0};
  }
  const bool current = slot == current_slot_;
  std::int64_t left = capacity(link, slot).value() -
                      (current ? used_in_current_slot_.at(link == Link::kUplink ? 0 : 1) : 0);
  if (medium_) {
    left =
        std::min(left, medium_->capacity(SlotIndex{slot}).value() - (current ? medium_used_ : 0));
  }
  return Bytes{left};
}

std::optional<Bytes> Engine::unused_by(Link link, RobotId robot, std::int64_t slot) const {
  const std::optional<Bytes>& cap =
      link == Link::kUplink ? environment_.endpoint_uplink : environment_.endpoint_downlink;
  if (!cap) {
    return std::nullopt;
  }
  const auto used = endpoint_used_.find({link, robot});
  const bool counted = slot == current_slot_ && used != endpoint_used_.end();
  return Bytes{cap->value() - (counted ? used->second : 0)};
}

TimePoint Engine::lane_free() const { return lane_free_; }

std::size_t Engine::size() const { return cohort_.size(); }

const Request& Engine::request(std::size_t index) const { return cohort_.at(index); }

const Progress& Engine::progress(std::size_t index) const { return progress_.at(index); }

std::int64_t Engine::slot_of(TimePoint time) const {
  return time.ns() / environment_.slot_length.ns();
}

void Engine::advance_to(std::int64_t slot) {
  if (slot != current_slot_) {
    used_in_current_slot_ = {};
    medium_used_ = 0;
    endpoint_used_.clear();
    current_slot_ = slot;
  }
}

bool Engine::servable(Link link, TimePoint start, const Grant& grant) const {
  if (grant.request >= cohort_.size() || grant.bytes <= Bytes{0}) {
    return false;
  }
  const Request& offered = cohort_.at(grant.request);
  const Progress& state = progress_.at(grant.request);
  if (state.dropped) {
    return false;
  }
  if (link == Link::kUplink) {
    const std::optional<TimePoint> sent = checked_add(offered.release, offered.send_delay);
    return sent && *sent <= start && grant.bytes <= state.upload_left;
  }
  return state.produced && *state.produced <= start && grant.bytes <= state.response_left;
}

bool Engine::serve(Link link, std::int64_t slot, const std::vector<Grant>& grants) {
  if (slot < current_slot_ || slot >= environment_.slot_count) {
    return false;
  }
  const TimePoint start = slot_start(slot);
  const TimePoint end = slot_start(slot + 1);
  const std::optional<TimePoint> done = checked_add(
      end, link == Link::kUplink ? environment_.uplink_delay : environment_.downlink_delay);
  if (!done) {
    return false;
  }
  std::vector<std::size_t> named;
  named.reserve(grants.size());
  std::ranges::transform(grants, std::back_inserter(named), &Grant::request);
  std::ranges::sort(named);
  if (std::ranges::adjacent_find(named) != named.end()) {
    return false;
  }
  const std::int64_t available = unused(link, slot).value();
  std::int64_t left = available;
  std::map<RobotId, std::int64_t> robot_left;
  for (const Grant& grant : grants) {
    if (!servable(link, start, grant) || grant.bytes.value() > left) {
      return false;
    }
    left -= grant.bytes.value();
    const RobotId robot = cohort_.at(grant.request).robot;
    const std::optional<Bytes> cap = unused_by(link, robot, slot);
    if (cap) {
      const auto entry = robot_left.try_emplace(robot, cap->value()).first;
      if (grant.bytes.value() > entry->second) {
        return false;
      }
      entry->second -= grant.bytes.value();
    }
  }
  advance_to(slot);
  used_in_current_slot_.at(link == Link::kUplink ? 0 : 1) += available - left;
  medium_used_ += available - left;
  for (const Grant& grant : grants) {
    endpoint_used_[{link, cohort_.at(grant.request).robot}] += grant.bytes.value();
    Progress& state = progress_.at(grant.request);
    Bytes& remaining = link == Link::kUplink ? state.upload_left : state.response_left;
    remaining = Bytes{remaining.value() - grant.bytes.value()};
    record(Event{
        .time = start,
        .kind = link == Link::kUplink ? EventKind::kUploadService : EventKind::kResponseService,
        .request = grant.request,
        .bytes = grant.bytes});
    if (remaining != Bytes{0}) {
      continue;
    }
    if (link == Link::kUplink) {
      state.ready = done;
      record(Event{.time = end, .kind = EventKind::kUploadComplete, .request = grant.request});
    } else {
      state.receipt = done;
      record(Event{.time = *done, .kind = EventKind::kReceipt, .request = grant.request});
    }
  }
  return true;
}

bool Engine::start_compute(std::size_t index, TimePoint at) {
  if (index >= cohort_.size()) {
    return false;
  }
  Progress& state = progress_.at(index);
  const std::optional<TimePoint> end = checked_add(at, environment_.compute_actual);
  if (!state.ready || *state.ready > at || state.compute_start || state.dropped ||
      at < lane_free_ || at >= horizon() || slot_of(at) < current_slot_ || !end) {
    return false;
  }
  advance_to(slot_of(at));
  state.compute_start = at;
  state.produced = end;
  lane_free_ = *end;
  record(Event{.time = at, .kind = EventKind::kComputeStart, .request = index});
  record(Event{.time = *end, .kind = EventKind::kComputeEnd, .request = index});
  return true;
}

bool Engine::drop(std::size_t index, TimePoint at) {
  if (index >= cohort_.size()) {
    return false;
  }
  Progress& state = progress_.at(index);
  if (state.dropped || state.receipt || at < TimePoint{} || at >= horizon() ||
      slot_of(at) < current_slot_) {
    return false;
  }
  advance_to(slot_of(at));
  state.dropped = at;
  record(Event{.time = at, .kind = EventKind::kDropped, .request = index});
  return true;
}

void Engine::record(Event event) {
  digest_.add(event.time.ns());
  digest_.add(std::uint64_t{static_cast<std::uint8_t>(event.kind)});
  digest_.add(cohort_.at(event.request).id.value());
  digest_.add(event.bytes.value());
  events_.push_back(event);
}

const std::vector<Event>& Engine::events() const { return events_; }

std::uint64_t Engine::digest() const { return digest_.value(); }

}  // namespace rovi::core
