// Authoritative actual-service ledger shared by every compared policy: byte service per slot on the
// uplink and downlink, one non-preemptive compute lane, and the actual stage times of each request.
#ifndef ROVI_CORE_ENGINE_HPP_
#define ROVI_CORE_ENGINE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/cohort.hpp"
#include "rovi_core/digest.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {

struct Environment {
  Duration slot_length;
  std::int64_t slot_count{0};
  std::vector<ServiceSegment> uplink;
  std::vector<ServiceSegment> downlink;
  Duration uplink_delay{};
  Duration downlink_delay{};
  Duration compute_planned;
  Duration compute_actual;
  // Optional budgets: a medium both directions share per slot, and per-robot caps per direction.
  std::optional<std::vector<ServiceSegment>> medium{};
  std::optional<Bytes> endpoint_uplink{};
  std::optional<Bytes> endpoint_downlink{};
};

enum class Link : std::uint8_t { kUplink, kDownlink };

struct Grant {
  std::size_t request{0};
  Bytes bytes;
};

enum class EventKind : std::uint8_t {
  kUploadService,
  kUploadComplete,
  kComputeStart,
  kComputeEnd,
  kResponseService,
  kReceipt,
  kDropped,
};

struct Event {
  TimePoint time;
  EventKind kind{EventKind::kUploadService};
  std::size_t request{0};
  Bytes bytes{};

  bool operator==(const Event&) const = default;
};

struct Progress {
  Bytes upload_left;
  Bytes response_left;
  std::optional<TimePoint> ready;
  std::optional<TimePoint> compute_start;
  std::optional<TimePoint> produced;
  std::optional<TimePoint> receipt;
  std::optional<TimePoint> dropped{};

  bool operator==(const Progress&) const = default;
};

// Data first receives service at the first slot boundary at or after it exists: an upload after
// its release, a response after its compute ends. A transfer completes at the end of the slot that
// serves its last byte, and its fixed delay follows. Time only moves forward: an action in a slot
// before the latest one acted in is refused, and a slot's capacity is shared by every grant made in
// it. Every action is validated before any of it is applied, so a refused action changes nothing.
class Engine {
 public:
  [[nodiscard]] static std::optional<Engine> create(Environment environment,
                                                    std::vector<Request> cohort);

  [[nodiscard]] TimePoint slot_start(std::int64_t slot) const;
  [[nodiscard]] TimePoint horizon() const;
  [[nodiscard]] Bytes capacity(Link link, std::int64_t slot) const;
  // What a grant in the slot may still use: the link and the medium, and the robot's own cap, which
  // is nullopt when that direction is uncapped.
  [[nodiscard]] Bytes unused(Link link, std::int64_t slot) const;
  [[nodiscard]] std::optional<Bytes> unused_by(Link link, RobotId robot, std::int64_t slot) const;
  [[nodiscard]] TimePoint lane_free() const;
  [[nodiscard]] std::size_t size() const;
  [[nodiscard]] const Request& request(std::size_t index) const;
  [[nodiscard]] const Progress& progress(std::size_t index) const;

  [[nodiscard]] bool serve(Link link, std::int64_t slot, const std::vector<Grant>& grants);
  [[nodiscard]] bool start_compute(std::size_t index, TimePoint at);
  // Ends a request's service for good; it is refused once received or already dropped.
  [[nodiscard]] bool drop(std::size_t index, TimePoint at);

  [[nodiscard]] const std::vector<Event>& events() const;
  [[nodiscard]] std::uint64_t digest() const;

 private:
  Engine(Environment environment, std::vector<Request> cohort, CapacityCalendar uplink,
         CapacityCalendar downlink, std::optional<CapacityCalendar> medium);

  [[nodiscard]] bool servable(Link link, TimePoint start, const Grant& grant) const;
  [[nodiscard]] std::int64_t slot_of(TimePoint time) const;
  void advance_to(std::int64_t slot);
  void record(Event event);

  Environment environment_;
  std::vector<Request> cohort_;
  std::vector<Progress> progress_;
  CapacityCalendar uplink_;
  CapacityCalendar downlink_;
  TimePoint lane_free_;
  std::optional<CapacityCalendar> medium_;
  std::int64_t current_slot_{0};
  std::array<std::int64_t, 2> used_in_current_slot_{};
  std::int64_t medium_used_{0};
  std::map<std::pair<Link, RobotId>, std::int64_t> endpoint_used_;
  std::vector<Event> events_;
  Fnv1aDigest digest_;
};

}  // namespace rovi::core

#endif  // ROVI_CORE_ENGINE_HPP_
