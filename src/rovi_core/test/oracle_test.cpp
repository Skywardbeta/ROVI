#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/admission.hpp"
#include "rovi_core/audit.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"

#include "printers.hpp"  // IWYU pragma: keep
#include "scenario.hpp"
#include "test_support.hpp"

namespace rovi::core {
namespace {

class Dice {
 public:
  explicit Dice(std::uint64_t seed) : engine_{seed} {}

  std::int64_t between(std::int64_t low, std::int64_t high) {
    const auto span = static_cast<std::uint64_t>(high - low + 1);
    return low + static_cast<std::int64_t>(engine_() % span);
  }

  bool chance(std::int64_t one_in) { return between(1, one_in) == 1; }

 private:
  std::mt19937_64 engine_;
};

struct Instance {
  ResourceSnapshot snapshot;
  std::vector<Job> jobs;
  std::vector<JobId> completed;
  TimePoint now;
};

CapacityCalendar random_calendar(Dice& dice, std::int64_t slots, std::int64_t max_kilobytes) {
  std::vector<Bytes> capacity;
  capacity.reserve(static_cast<std::size_t>(slots));
  for (std::int64_t slot = 0; slot < slots; ++slot) {
    capacity.emplace_back(dice.between(0, max_kilobytes) * 1'000);
  }
  return CapacityCalendar::create(TimePoint{}, ms(1), capacity).value();
}

Job random_job(Dice& dice, std::uint64_t id, std::int64_t worker_count, std::int64_t job_count) {
  Job job = job_numbered(id);
  job.buffer_empty_time = at_ms(dice.between(5, 40));
  job.safety_margin = ms(dice.between(0, 3));
  job.maximum_result_age =
      dice.chance(4) ? std::nullopt : std::optional<Duration>{ms(dice.between(5, 40))};
  job.upload_bytes = Bytes{dice.between(1, 120) * 1'000};
  job.action_chunk_bytes = Bytes{dice.between(1, 60) * 1'000};
  job.model = ModelClass{static_cast<std::uint32_t>(dice.between(0, 1))};
  job.eligible_workers.clear();
  for (std::int64_t worker_id = 0; worker_id < worker_count; ++worker_id) {
    if (dice.chance(2)) {
      job.eligible_workers.emplace_back(static_cast<std::uint64_t>(worker_id));
    }
  }
  if (job.eligible_workers.empty()) {
    job.eligible_workers.emplace_back(0U);
  }
  if (dice.chance(3)) {
    job.synchronization_group = GroupId{static_cast<std::uint32_t>(dice.between(1, 2))};
    job.maximum_result_ready_skew = ms(dice.between(0, 15));
  }
  if (dice.chance(4)) {
    job.predecessors = {JobId{static_cast<std::uint64_t>(dice.between(1, job_count + 2))}};
  }
  return job;
}

Instance random_instance(std::uint64_t seed) {
  Dice dice{seed};
  constexpr std::int64_t kSlots = 40;
  Instance instance{.snapshot = ResourceSnapshot{.uplink = random_calendar(dice, kSlots, 60),
                                                 .downlink = random_calendar(dice, kSlots, 40),
                                                 .workers = {}},
                    .jobs = {},
                    .completed = {},
                    .now = at_ms(dice.between(0, 3))};
  if (dice.chance(2)) {
    instance.snapshot.shared_medium = random_calendar(dice, kSlots, 90);
  }
  instance.snapshot.uplink_delay = ms(dice.between(0, 2));
  instance.snapshot.downlink_delay = ms(dice.between(0, 2));
  const std::int64_t worker_count = dice.between(1, 3);
  for (std::int64_t id = 0; id < worker_count; ++id) {
    instance.snapshot.workers.push_back(
        worker_with(static_cast<std::uint64_t>(id),
                    {{.model = ModelClass{0}, .duration = ms(dice.between(1, 6))},
                     {.model = ModelClass{1}, .duration = ms(dice.between(2, 10))}}));
  }
  const std::int64_t job_count = dice.between(1, 6);
  for (std::int64_t id = 1; id <= job_count; ++id) {
    instance.jobs.push_back(
        random_job(dice, static_cast<std::uint64_t>(id), worker_count, job_count));
  }
  for (std::int64_t id = 1; id <= job_count + 2; ++id) {
    if (dice.chance(2)) {
      instance.completed.emplace_back(static_cast<std::uint64_t>(id));
    }
  }
  return instance;
}

bool reports(const std::vector<AuditFinding>& findings, Violation violation) {
  return std::ranges::any_of(findings, [violation](const AuditFinding& finding) {
    return finding.violation == violation;
  });
}

TEST(RandomizedSchedules, SatisfyEveryInvariantUnderEveryPolicy) {
  std::int64_t admitted_jobs = 0;
  std::int64_t admitted_group_members = 0;
  for (std::uint64_t seed = 1; seed <= 300; ++seed) {
    SCOPED_TRACE(::testing::Message() << "seed " << seed);
    const Instance instance = random_instance(seed);
    for (const Policy policy :
         {Policy::kLeastSlack, Policy::kEarliestDeadline, Policy::kFirstInFirstOut}) {
      const ScheduleResult result =
          schedule(instance.snapshot, instance.jobs, instance.completed, instance.now, policy);
      const ScheduleResult repeat =
          schedule(instance.snapshot, instance.jobs, instance.completed, instance.now, policy);

      EXPECT_TRUE(
          audit_schedule(instance.snapshot, instance.jobs, instance.completed, result).empty());
      EXPECT_EQ(result.decisions.size(), instance.jobs.size());
      EXPECT_EQ(repeat.digest, result.digest);
      for (const JobDecision& decision : result.decisions) {
        if (decision.decision.status == DecisionStatus::kAdmitted) {
          ++admitted_jobs;
          const auto job = std::ranges::find(instance.jobs, decision.job, &Job::id);
          admitted_group_members += job->synchronization_group.has_value() ? 1 : 0;
        }
      }
    }
  }
  EXPECT_GT(admitted_jobs, 300);
  EXPECT_GT(admitted_group_members, 30);
}

constexpr std::int64_t kRobots = 30;
constexpr std::int64_t kTeamSize = 3;
constexpr std::int64_t kBurstSlots = 400;

CapacityCalendar calendar_with_outage(std::int64_t bytes_per_slot, std::int64_t outage_start,
                                      std::int64_t outage_end) {
  std::vector<Bytes> capacity;
  capacity.reserve(static_cast<std::size_t>(kBurstSlots));
  for (std::int64_t slot = 0; slot < kBurstSlots; ++slot) {
    capacity.emplace_back(slot >= outage_start && slot < outage_end ? 0 : bytes_per_slot);
  }
  return CapacityCalendar::create(TimePoint{}, ms(1), capacity).value();
}

Instance workcell_burst(std::uint64_t seed) {
  Dice dice{seed};
  const std::int64_t outage_start = dice.between(0, kBurstSlots);
  const std::int64_t outage_end = outage_start + dice.between(0, 150);
  Instance instance{
      .snapshot =
          ResourceSnapshot{
              .uplink = calendar_with_outage(11'250, outage_start, outage_end),
              .downlink = calendar_with_outage(5'625, outage_start, outage_end),
              .workers = {worker_with(0, {{.model = ModelClass{1}, .duration = ms(15)},
                                          {.model = ModelClass{2}, .duration = ms(30)}}),
                          worker_for(1, ModelClass{1}, ms(15)),
                          worker_for(2, ModelClass{2}, ms(30))}},
      .jobs = {},
      .completed = {},
      .now = at_ms(dice.between(0, 5))};
  instance.snapshot.uplink_delay = ms(2);
  instance.snapshot.downlink_delay = ms(2);
  for (std::int64_t robot = 0; robot < kRobots; ++robot) {
    const bool first_half = robot < kRobots / 2;
    Job job = job_numbered(static_cast<std::uint64_t>(robot + 1));
    job.robot = RobotId{static_cast<std::uint32_t>(robot)};
    job.buffer_empty_time = at_ms(dice.between(150, 600));
    job.safety_margin = ms(dice.between(0, 20));
    job.maximum_result_age =
        dice.chance(3) ? std::nullopt : std::optional<Duration>{ms(dice.between(300, 800))};
    job.upload_bytes = Bytes{dice.between(60, 140) * 1'000};
    job.action_chunk_bytes = Bytes{dice.between(10, 30) * 1'000};
    job.model = ModelClass{first_half ? 1U : 2U};
    job.eligible_workers = {WorkerId{0}, WorkerId{first_half ? 1U : 2U}};
    job.synchronization_group = GroupId{static_cast<std::uint32_t>((robot / kTeamSize) + 1)};
    job.maximum_result_ready_skew = ms(dice.between(20, 80));
    instance.jobs.push_back(job);
  }
  return instance;
}

TEST(RandomizedSchedules, ThirtyRobotWorkcellBurstsSatisfyEveryInvariantUnderEveryPolicy) {
  std::int64_t admitted_groups = 0;
  std::int64_t refused_groups = 0;
  for (std::uint64_t seed = 1; seed <= 40; ++seed) {
    SCOPED_TRACE(::testing::Message() << "seed " << seed);
    const Instance instance = workcell_burst(seed);
    std::vector<Job> shuffled = instance.jobs;
    std::ranges::shuffle(shuffled, std::mt19937_64{seed});
    for (const Policy policy :
         {Policy::kLeastSlack, Policy::kEarliestDeadline, Policy::kFirstInFirstOut}) {
      const ScheduleResult result =
          schedule(instance.snapshot, instance.jobs, instance.completed, instance.now, policy);
      const ScheduleResult reordered =
          schedule(instance.snapshot, shuffled, instance.completed, instance.now, policy);

      EXPECT_TRUE(
          audit_schedule(instance.snapshot, instance.jobs, instance.completed, result).empty());
      ASSERT_EQ(result.decisions.size(), instance.jobs.size());
      EXPECT_EQ(reordered.digest, result.digest);
      for (std::int64_t team = 0; team < kRobots / kTeamSize; ++team) {
        std::int64_t admitted_members = 0;
        for (const JobDecision& decision : result.decisions) {
          const auto job = std::ranges::find(instance.jobs, decision.job, &Job::id);
          ASSERT_NE(job, instance.jobs.end());
          if (job->synchronization_group == GroupId{static_cast<std::uint32_t>(team + 1)} &&
              decision.decision.status == DecisionStatus::kAdmitted) {
            ++admitted_members;
          }
        }
        EXPECT_TRUE(admitted_members == 0 || admitted_members == kTeamSize) << "team " << team;
        admitted_groups += admitted_members == kTeamSize ? 1 : 0;
        refused_groups += admitted_members == 0 ? 1 : 0;
      }
    }
  }
  EXPECT_GT(admitted_groups, 100);
  EXPECT_GT(refused_groups, 100);
}

class AuditTamper : public ::testing::Test {
 protected:
  ResourceSnapshot snapshot_ = empty_snapshot({worker(0, ms(5)), worker(1, ms(5))});
  std::vector<Job> jobs_ = {job_numbered(1), job_numbered(2)};
  ScheduleResult result_ = schedule(snapshot_, jobs_, {}, at_ms(0), Policy::kLeastSlack);

  ChainReservation& chain(std::size_t index) {
    return result_.decisions.at(index).reservation.value();
  }

  [[nodiscard]] std::vector<AuditFinding> audit() const {
    return audit_schedule(snapshot_, jobs_, {}, result_);
  }
};

TEST_F(AuditTamper, AcceptsTheUntamperedSchedule) {
  ASSERT_EQ(result_.decisions.at(0).decision, admitted());
  ASSERT_EQ(result_.decisions.at(1).decision, admitted());
  EXPECT_TRUE(audit().empty());
}

TEST_F(AuditTamper, DetectsDigestMismatch) {
  result_.digest ^= 1U;
  EXPECT_TRUE(reports(audit(), Violation::kDigestMismatch));
}

TEST_F(AuditTamper, DetectsComputeStartingBeforeUploadCompletes) {
  chain(0).compute = window_ms(1, 6);
  EXPECT_TRUE(reports(audit(), Violation::kStageOrder));
}

TEST(ScheduleAudit, DetectsComputeStartingWithinTheDispatchLead) {
  ResourceSnapshot snapshot = empty_snapshot({worker(0, ms(5))});
  snapshot.dispatch_lead = ms(1);
  const std::vector<Job> jobs = {job_numbered(1)};
  ScheduleResult result = schedule(snapshot, jobs, {}, at_ms(0), Policy::kLeastSlack);
  ASSERT_EQ(result.decisions.at(0).decision, admitted());
  EXPECT_TRUE(audit_schedule(snapshot, jobs, {}, result).empty());

  result.decisions.at(0).reservation->compute = window_ms(3, 8);

  EXPECT_TRUE(reports(audit_schedule(snapshot, jobs, {}, result), Violation::kStageOrder));
}

TEST_F(AuditTamper, DetectsCapacityOverbooking) {
  chain(0).upload.slices.at(0).bytes = Bytes{200'000};
  EXPECT_TRUE(reports(audit(), Violation::kCapacityExceeded));
}

TEST_F(AuditTamper, DetectsComputeLengthDifferentFromWorkerProfile) {
  chain(0).compute = window_ms(2, 8);
  EXPECT_TRUE(reports(audit(), Violation::kComputeLengthMismatch));
}

TEST_F(AuditTamper, DetectsOverlappingComputeOnOneLane) {
  chain(1).worker = chain(0).worker;
  chain(1).compute = chain(0).compute;
  EXPECT_TRUE(reports(audit(), Violation::kComputeOverlap));
}

TEST_F(AuditTamper, DetectsDeliveryInconsistentWithDownload) {
  chain(0).delivery = at_ms(30);
  EXPECT_TRUE(reports(audit(), Violation::kDeliveryMismatch));
}

TEST_F(AuditTamper, DetectsUnknownWorkerAndJob) {
  chain(0).worker = WorkerId{9};
  result_.decisions.at(1).job = JobId{99};
  const std::vector<AuditFinding> findings = audit();
  EXPECT_TRUE(reports(findings, Violation::kUnknownWorker));
  EXPECT_TRUE(reports(findings, Violation::kUnknownJob));
}

TEST_F(AuditTamper, DetectsSnapshotThatDoesNotMatchAdmittedReservations) {
  result_.snapshot = snapshot_;
  EXPECT_TRUE(reports(audit(), Violation::kSnapshotMismatch));
}

TEST_F(AuditTamper, DetectsDeadlineExceededForTheAuditedJobs) {
  jobs_.at(0).buffer_empty_time = at_ms(6);
  EXPECT_TRUE(reports(audit(), Violation::kDeadlineExceeded));
}

TEST_F(AuditTamper, DetectsPredecessorThatHadNotCompleted) {
  jobs_.at(1).predecessors = {JobId{1}};
  EXPECT_TRUE(reports(audit(), Violation::kPredecessorIncomplete));
}

TEST_F(AuditTamper, DetectsSliceOutsideTheHorizon) {
  chain(0).upload.slices.at(0).slot = SlotIndex{500};
  EXPECT_TRUE(reports(audit(), Violation::kCapacityExceeded));
}

TEST_F(AuditTamper, DetectsChainWithoutDownloadService) {
  chain(0).download.slices.clear();
  const std::vector<AuditFinding> findings = audit();
  EXPECT_TRUE(reports(findings, Violation::kStageOrder));
  EXPECT_TRUE(reports(findings, Violation::kDeliveryMismatch));
}

TEST_F(AuditTamper, DetectsResultCalendarWithDifferentHorizon) {
  result_.snapshot.uplink = uniform_calendar(kUplinkBytesPerSlot, 50);
  EXPECT_TRUE(reports(audit(), Violation::kSnapshotMismatch));
}

TEST_F(AuditTamper, DetectsPartialGroupAdmissionAndExcessiveSkew) {
  jobs_.at(0).synchronization_group = GroupId{4};
  jobs_.at(1).synchronization_group = GroupId{4};
  EXPECT_TRUE(reports(audit(), Violation::kGroupSkewExceeded));

  result_.decisions.at(1).decision =
      outcome(DecisionStatus::kDeferred, ReasonCode::kNoFeasibleChain);
  result_.decisions.at(1).reservation.reset();
  EXPECT_TRUE(reports(audit(), Violation::kGroupNotAtomic));
}

std::optional<std::int64_t> greedy_completion_ms(const std::vector<std::int64_t>& capacity,
                                                 std::int64_t first_slot, std::int64_t demand) {
  std::int64_t remaining = demand;
  for (std::int64_t slot = first_slot; slot < std::ssize(capacity); ++slot) {
    remaining -= std::min(remaining, capacity.at(static_cast<std::size_t>(slot)));
    if (remaining == 0) {
      return slot + 1;
    }
  }
  return std::nullopt;
}

bool lane_free(const std::vector<std::pair<std::int64_t, std::int64_t>>& busy, std::int64_t start,
               std::int64_t end) {
  return std::ranges::none_of(busy, [start, end](const auto& interval) {
    return start < interval.second && interval.first < end;
  });
}

std::optional<std::int64_t> brute_force_delivery_ms(
    const std::vector<std::int64_t>& up, const std::vector<std::int64_t>& down,
    const std::vector<std::pair<std::int64_t, std::int64_t>>& busy, std::int64_t now,
    std::int64_t duration, std::int64_t upload, std::int64_t chunk) {
  std::optional<std::int64_t> best;
  const std::int64_t slots = std::ssize(up);
  for (std::int64_t upload_slot = now; upload_slot < slots; ++upload_slot) {
    const std::optional<std::int64_t> uploaded = greedy_completion_ms(up, upload_slot, upload);
    for (std::int64_t start = uploaded.value_or(slots + 1); start <= slots; ++start) {
      if (!lane_free(busy, start, start + duration)) {
        continue;
      }
      for (std::int64_t download_slot = start + duration; download_slot < slots; ++download_slot) {
        const std::optional<std::int64_t> delivered =
            greedy_completion_ms(down, download_slot, chunk);
        if (delivered && (!best || *delivered < *best)) {
          best = delivered;
        }
      }
    }
  }
  return best;
}

TEST(EarliestDeliveryOracle, AdmissionMatchesBruteForceOnTinyInstances) {
  std::int64_t admitted_cases = 0;
  for (std::uint64_t seed = 1; seed <= 300; ++seed) {
    SCOPED_TRACE(::testing::Message() << "seed " << seed);
    Dice dice{seed};
    constexpr std::int64_t kSlots = 12;
    std::vector<std::int64_t> up;
    std::vector<std::int64_t> down;
    for (std::int64_t slot = 0; slot < kSlots; ++slot) {
      up.push_back(dice.between(0, 3) * 10);
      down.push_back(dice.between(0, 3) * 10);
    }
    std::vector<std::pair<std::int64_t, std::int64_t>> busy;
    IntervalCalendar lane;
    for (std::int64_t start = 0; start < kSlots; start += 3) {
      if (dice.chance(3)) {
        const std::int64_t end = start + dice.between(1, 2);
        busy.emplace_back(start, end);
        lane = lane.reserve(window_ms(start, end)).value();
      }
    }
    const std::int64_t duration = dice.between(1, 3);
    const std::int64_t upload = dice.between(1, 6) * 10;
    const std::int64_t chunk = dice.between(1, 6) * 10;
    const std::int64_t now = dice.between(0, 2);
    std::vector<Bytes> up_bytes;
    std::vector<Bytes> down_bytes;
    for (std::int64_t slot = 0; slot < kSlots; ++slot) {
      up_bytes.emplace_back(up.at(static_cast<std::size_t>(slot)));
      down_bytes.emplace_back(down.at(static_cast<std::size_t>(slot)));
    }
    Worker only = worker(0, ms(duration));
    only.lane = lane;
    const ResourceSnapshot snapshot{
        .uplink = CapacityCalendar::create(TimePoint{}, ms(1), up_bytes).value(),
        .downlink = CapacityCalendar::create(TimePoint{}, ms(1), down_bytes).value(),
        .workers = {only}};
    Job job = reference_job();
    job.buffer_empty_time = at_ms(1'000);
    job.maximum_result_age = ms(1'000);
    job.upload_bytes = Bytes{upload};
    job.action_chunk_bytes = Bytes{chunk};

    const AdmissionResult result = admit(snapshot, job, at_ms(now));
    const std::optional<std::int64_t> expected =
        brute_force_delivery_ms(up, down, busy, now, duration, upload, chunk);

    if (expected.has_value()) {
      ++admitted_cases;
      ASSERT_EQ(result.decision, admitted());
      EXPECT_EQ(result.reservation.value().delivery, at_ms(*expected));
    } else {
      EXPECT_EQ(result.decision.status, DecisionStatus::kDeferred);
    }
  }
  EXPECT_GT(admitted_cases, 100);
}

}  // namespace
}  // namespace rovi::core
