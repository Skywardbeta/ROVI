#include "rovi_core/engine.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rovi_core/cohort.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

constexpr std::int64_t kMillisecond = 1'000'000;

TimePoint at_ms(std::int64_t ms) { return TimePoint::from_ns(ms * kMillisecond); }

// 80 Mbps over 1 ms slots is 10,000 bytes per slot in each direction.
Environment environment() {
  const std::vector<ServiceSegment> link{
      {.first_slot = 0, .rate = BitsPerSecond{80'000'000}, .reduction = Permille{0}}};
  return Environment{.slot_length = Duration::from_ns(kMillisecond),
                     .slot_count = 100,
                     .uplink = link,
                     .downlink = link,
                     .uplink_delay = Duration::from_ns(2 * kMillisecond),
                     .downlink_delay = Duration::from_ns(3 * kMillisecond),
                     .compute_planned = Duration::from_ns(10 * kMillisecond),
                     .compute_actual = Duration::from_ns(10 * kMillisecond)};
}

Request request(std::uint32_t robot, TimePoint release, std::int64_t upload = 20'000,
                std::int64_t response = 10'000) {
  return Request{.id = JobId{std::uint64_t{robot} << 32U},
                 .robot = RobotId{robot},
                 .release = release,
                 .deadline = at_ms(90),
                 .upload = Bytes{upload},
                 .response = Bytes{response}};
}

Engine engine_for(std::vector<Request> cohort, Environment setting = environment()) {
  std::optional<Engine> engine = Engine::create(std::move(setting), std::move(cohort));
  if (!engine) {
    throw std::invalid_argument{"the test engine was refused"};
  }
  return std::move(*engine);
}

std::vector<Grant> grant(std::size_t index, std::int64_t bytes) {
  return {Grant{.request = index, .bytes = Bytes{bytes}}};
}

TEST(Engine, ServesAnUploadOnlyFromTheFirstBoundaryAfterItsRelease) {
  Engine engine = engine_for({request(0, TimePoint::from_ns(kMillisecond + 1))});
  EXPECT_EQ(engine.capacity(Link::kUplink, 1), Bytes{10'000});
  EXPECT_FALSE(engine.serve(Link::kUplink, 1, grant(0, 10'000)));
  EXPECT_TRUE(engine.events().empty());
  EXPECT_EQ(engine.progress(0).upload_left, Bytes{20'000});
  EXPECT_TRUE(engine.serve(Link::kUplink, 2, grant(0, 10'000)));
  EXPECT_EQ(engine.progress(0).upload_left, Bytes{10'000});
}

TEST(Engine, RefusesGrantsBeyondTheSlotCapacityOrTheRemainingBytes) {
  Engine engine = engine_for({request(0, at_ms(0)), request(1, at_ms(0), 5'000)});
  const std::vector<Grant> oversubscribed{{.request = 0, .bytes = Bytes{6'000}},
                                          {.request = 1, .bytes = Bytes{5'000}}};
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, oversubscribed));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(1, 5'001)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(1, 0)));
  const std::vector<Grant> twice{{.request = 1, .bytes = Bytes{1'000}},
                                 {.request = 1, .bytes = Bytes{1'000}}};
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, twice));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(2, 1'000)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 100, grant(0, 1'000)));
  EXPECT_TRUE(engine.events().empty());
  const std::vector<Grant> shared{{.request = 0, .bytes = Bytes{5'000}},
                                  {.request = 1, .bytes = Bytes{5'000}}};
  EXPECT_TRUE(engine.serve(Link::kUplink, 0, shared));
}

TEST(Engine, AccountsForEachSlotAcrossCallsAndNeverActsInThePast) {
  Engine engine = engine_for({request(0, at_ms(0)), request(1, at_ms(0)),
                              request(2, at_ms(0), 10'000), request(3, at_ms(0), 10'000)});
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 6'000)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(1, 5'000)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(1, 4'000)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(1, 1)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 2, grant(2, 10'000)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 1, grant(3, 10'000)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 7, grant(0, 10'000)));
  EXPECT_EQ(engine.progress(2).ready, at_ms(5));
  EXPECT_FALSE(engine.start_compute(2, at_ms(6)));
  EXPECT_TRUE(engine.start_compute(2, at_ms(7)));
}

TEST(Engine, ARefusedBatchLeavesProgressResidualCapacityAndEventsUnchanged) {
  Engine engine = engine_for({request(0, at_ms(0)), request(1, at_ms(0))});
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 4'000)));
  const Progress progress = engine.progress(0);
  const std::vector<Event> events = engine.events();
  const std::uint64_t digest = engine.digest();
  const std::vector<Grant> with_unknown{{.request = 0, .bytes = Bytes{1'000}},
                                        {.request = 2, .bytes = Bytes{1'000}}};
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, with_unknown));
  EXPECT_EQ(engine.progress(0), progress);
  EXPECT_EQ(engine.events(), events);
  EXPECT_EQ(engine.digest(), digest);
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(1, 6'001)));
  EXPECT_TRUE(engine.serve(Link::kUplink, 0, grant(1, 6'000)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(0, 1)));
  EXPECT_TRUE(engine.serve(Link::kUplink, 1, grant(0, 1)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(0, 1)));
}

TEST(Engine, CompletesAStageOnlyAfterEveryByteIsServedAndThenAddsItsDelay) {
  Engine engine = engine_for({request(0, at_ms(0), 15'000)});
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 10'000)));
  EXPECT_FALSE(engine.progress(0).ready.has_value());
  ASSERT_TRUE(engine.serve(Link::kUplink, 3, grant(0, 5'000)));
  EXPECT_EQ(engine.progress(0).ready, at_ms(6));
  EXPECT_FALSE(engine.serve(Link::kUplink, 4, grant(0, 1)));
}

TEST(Engine, ComputeWaitsForTheCompleteObservationAndNeverOverlaps) {
  Engine engine = engine_for({request(0, at_ms(0), 10'000), request(1, at_ms(0), 10'000)});
  EXPECT_FALSE(engine.start_compute(0, at_ms(0)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 10'000)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 1, grant(1, 10'000)));
  EXPECT_FALSE(engine.start_compute(0, TimePoint::from_ns((3 * kMillisecond) - 1)));
  ASSERT_TRUE(engine.start_compute(0, at_ms(3)));
  EXPECT_FALSE(engine.start_compute(0, at_ms(30)));
  EXPECT_FALSE(engine.start_compute(1, TimePoint::from_ns((13 * kMillisecond) - 1)));
  ASSERT_TRUE(engine.start_compute(1, at_ms(13)));
  EXPECT_EQ(engine.progress(0).produced, at_ms(13));
  EXPECT_EQ(engine.progress(1).produced, at_ms(23));
  EXPECT_EQ(engine.lane_free(), at_ms(23));
}

TEST(Engine, ServesAResponseOnlyFromTheBoundaryAfterItsComputeEnds) {
  Environment slower = environment();
  slower.compute_actual = Duration::from_ns((10 * kMillisecond) + (kMillisecond / 2));
  Engine engine = engine_for({request(0, at_ms(0), 10'000)}, slower);
  EXPECT_FALSE(engine.serve(Link::kDownlink, 0, grant(0, 10'000)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 10'000)));
  ASSERT_TRUE(engine.start_compute(0, at_ms(3)));
  EXPECT_FALSE(engine.serve(Link::kDownlink, 13, grant(0, 10'000)));
  ASSERT_TRUE(engine.serve(Link::kDownlink, 14, grant(0, 10'000)));
  EXPECT_EQ(engine.progress(0).receipt, at_ms(18));
}

TEST(Engine, ChangingTheAllocationChangesTheActualCompletion) {
  Engine first_come = engine_for({request(0, at_ms(0)), request(1, at_ms(0))});
  Engine shared = engine_for({request(0, at_ms(0)), request(1, at_ms(0))});
  for (std::int64_t slot = 0; slot < 4; ++slot) {
    ASSERT_TRUE(first_come.serve(Link::kUplink, slot, grant(slot < 2 ? 0 : 1, 10'000)));
    const std::vector<Grant> halves{{.request = 0, .bytes = Bytes{5'000}},
                                    {.request = 1, .bytes = Bytes{5'000}}};
    ASSERT_TRUE(shared.serve(Link::kUplink, slot, halves));
  }
  EXPECT_EQ(first_come.progress(0).ready, at_ms(4));
  EXPECT_EQ(first_come.progress(1).ready, at_ms(6));
  EXPECT_EQ(shared.progress(0).ready, at_ms(6));
  EXPECT_EQ(shared.progress(1).ready, at_ms(6));
  EXPECT_NE(first_come.digest(), shared.digest());
}

TEST(Engine, IdenticalActionsGiveIdenticalEventsAndDigests) {
  Engine first = engine_for({request(0, at_ms(0), 10'000)});
  Engine second = engine_for({request(0, at_ms(0), 10'000)});
  for (Engine* engine : {&first, &second}) {
    ASSERT_TRUE(engine->serve(Link::kUplink, 0, grant(0, 10'000)));
    ASSERT_TRUE(engine->start_compute(0, at_ms(3)));
    ASSERT_TRUE(engine->serve(Link::kDownlink, 13, grant(0, 10'000)));
  }
  EXPECT_EQ(first.events(), second.events());
  EXPECT_EQ(first.digest(), second.digest());
  EXPECT_EQ(first.events().size(), 6U);
}

// The slot has 10,000 bytes, but robot 0 may use only 4,000 of them
// across all its requests, in every grant made in the slot.
TEST(Engine, AnEndpointCapBindsTheSumOfOneRobotsRequests) {
  Environment capped = environment();
  capped.endpoint_uplink = Bytes{4'000};
  const Request first = request(0, at_ms(0));
  Request second = request(0, at_ms(0));
  second.id = JobId{1};
  Engine engine = engine_for({first, second, request(1, at_ms(0))}, capped);
  const std::vector<Grant> both_full{{.request = 0, .bytes = Bytes{4'000}},
                                     {.request = 1, .bytes = Bytes{4'000}}};
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, both_full));
  const std::vector<Grant> both_half{{.request = 0, .bytes = Bytes{2'000}},
                                     {.request = 1, .bytes = Bytes{2'000}}};
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, both_half));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(1, 1)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 0, grant(2, 4'001)));
  EXPECT_TRUE(engine.serve(Link::kUplink, 0, grant(2, 4'000)));
  EXPECT_TRUE(engine.serve(Link::kUplink, 1, grant(0, 4'000)));
}

// The uplink and downlink each offer 10,000 bytes per slot but share a 10,000-byte medium.
TEST(Engine, ACoupledMediumBindsBothDirectionsTogether) {
  Environment coupled = environment();
  coupled.medium = std::vector<ServiceSegment>{
      {.first_slot = 0, .rate = BitsPerSecond{80'000'000}, .reduction = Permille{0}}};
  Engine engine = engine_for({request(0, at_ms(0), 10'000), request(1, at_ms(0))}, coupled);
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 10'000)));
  ASSERT_TRUE(engine.start_compute(0, at_ms(3)));
  ASSERT_TRUE(engine.serve(Link::kUplink, 13, grant(1, 6'000)));
  EXPECT_FALSE(engine.serve(Link::kDownlink, 13, grant(0, 5'000)));
  EXPECT_TRUE(engine.serve(Link::kDownlink, 13, grant(0, 4'000)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 13, grant(1, 1)));
}

TEST(Engine, AnUploadIsServedOnlyAfterTheRobotActuallySendsIt) {
  Request late = request(0, at_ms(0));
  late.send_delay = Duration::from_ns(kMillisecond + (kMillisecond / 2));
  Engine engine = engine_for({late});
  EXPECT_FALSE(engine.serve(Link::kUplink, 1, grant(0, 10'000)));
  EXPECT_TRUE(engine.serve(Link::kUplink, 2, grant(0, 10'000)));
}

TEST(Engine, ADroppedRequestIsRecordedAndReceivesNoFurtherService) {
  Engine engine = engine_for({request(0, at_ms(0))});
  ASSERT_TRUE(engine.serve(Link::kUplink, 0, grant(0, 10'000)));
  ASSERT_TRUE(engine.drop(0, at_ms(1)));
  EXPECT_FALSE(engine.drop(0, at_ms(2)));
  EXPECT_FALSE(engine.serve(Link::kUplink, 2, grant(0, 10'000)));
  EXPECT_EQ(
      engine.events().back(),
      (Event{.time = at_ms(1), .kind = EventKind::kDropped, .request = 0, .bytes = Bytes{0}}));
  EXPECT_EQ(engine.progress(0).dropped, at_ms(1));
}

TEST(Engine, RefusesActionsOnARequestOrSlotItDoesNotHave) {
  Engine engine = engine_for({request(0, at_ms(0))});
  EXPECT_FALSE(engine.drop(1, at_ms(0)));
  EXPECT_FALSE(engine.start_compute(1, at_ms(0)));
  EXPECT_EQ(engine.capacity(Link::kUplink, -1), Bytes{0});
  EXPECT_EQ(engine.capacity(Link::kUplink, 100), Bytes{0});
  EXPECT_EQ(engine.unused(Link::kDownlink, 100), Bytes{0});
  EXPECT_TRUE(engine.events().empty());
  ASSERT_TRUE(engine.serve(Link::kUplink, 1, grant(0, 4'000)));
  EXPECT_EQ(engine.unused(Link::kUplink, 0), Bytes{0});
  EXPECT_EQ(engine.unused(Link::kUplink, 1), Bytes{6'000});
}

TEST(Engine, RejectsInvalidEnvironmentsAndCohorts) {
  EXPECT_TRUE(Engine::create(environment(), {request(0, at_ms(0))}).has_value());
  Environment bad = environment();
  bad.slot_length = Duration{};
  EXPECT_FALSE(Engine::create(bad, {request(0, at_ms(0))}).has_value());
  bad = environment();
  bad.slot_count = 0;
  EXPECT_FALSE(Engine::create(bad, {request(0, at_ms(0))}).has_value());
  bad = environment();
  bad.compute_actual = Duration{};
  EXPECT_FALSE(Engine::create(bad, {request(0, at_ms(0))}).has_value());
  bad = environment();
  bad.compute_planned = Duration{};
  EXPECT_FALSE(Engine::create(bad, {request(0, at_ms(0))}).has_value());
  bad = environment();
  bad.uplink_delay = Duration::from_ns(-1);
  EXPECT_FALSE(Engine::create(bad, {request(0, at_ms(0))}).has_value());
  bad = environment();
  bad.downlink.clear();
  EXPECT_FALSE(Engine::create(bad, {request(0, at_ms(0))}).has_value());
  EXPECT_FALSE(Engine::create(environment(), {}).has_value());
  EXPECT_FALSE(Engine::create(environment(), {request(0, at_ms(0)), request(0, at_ms(1))}));
  EXPECT_FALSE(Engine::create(environment(), {request(0, at_ms(0), 0)}).has_value());
  EXPECT_FALSE(Engine::create(environment(), {request(0, TimePoint::from_ns(-1))}).has_value());
}

}  // namespace
}  // namespace rovi::core
