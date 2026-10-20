#include "rovi_core/window.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/capacity_calendar.hpp"
#include "rovi_core/interval_calendar.hpp"
#include "rovi_core/units.hpp"
#include "rovi_core/wireless_profile.hpp"

namespace rovi::core {
namespace {

bool valid_workers(const std::vector<WorkerSpec>& workers) {
  std::set<WorkerId> ids;
  return std::ranges::all_of(workers, [&ids](const WorkerSpec& worker) {
    std::set<ModelClass> models;
    return ids.insert(worker.id).second &&
           std::ranges::all_of(worker.profiles, [&models](const ExecutionProfile& profile) {
             return profile.duration.ns() > 0 && models.insert(profile.model).second;
           });
  });
}

std::optional<CapacityCalendar> calendar_at(const std::vector<ServiceSegment>& segments,
                                            Duration slot_length, TimePoint epoch,
                                            std::int64_t first_slot, std::int64_t slot_count) {
  std::optional<std::vector<Bytes>> capacity =
      service_capacity(slot_length, segments, first_slot, slot_count);
  std::int64_t offset_ns{};
  if (!capacity || __builtin_mul_overflow(first_slot, slot_length.ns(), &offset_ns)) {
    return std::nullopt;
  }
  const std::optional<TimePoint> origin = checked_add(epoch, Duration::from_ns(offset_ns));
  if (!origin) {
    return std::nullopt;
  }
  return CapacityCalendar::create(*origin, slot_length, std::move(*capacity));
}

std::optional<CapacityCalendar> slid(const CapacityCalendar& calendar,
                                     const std::vector<ServiceSegment>& segments, TimePoint epoch,
                                     std::int64_t current_first, std::int64_t target_first) {
  const std::int64_t count = calendar.slot_count();
  const std::int64_t shift = target_first - current_first;
  if (shift >= count) {
    return calendar_at(segments, calendar.slot_length(), epoch, target_first, count);
  }
  std::optional<std::vector<Bytes>> appended =
      service_capacity(calendar.slot_length(), segments, current_first + count, shift);
  if (!appended) {
    return std::nullopt;
  }
  return calendar.advance(SlotIndex{shift}, *appended);
}

std::optional<std::int64_t> first_slot_of(const CapacityCalendar& calendar, TimePoint epoch) {
  const std::optional<TimePoint> origin = calendar.slot_start(SlotIndex{0});
  if (!origin) {
    return std::nullopt;
  }
  const std::optional<Duration> offset = elapsed(epoch, *origin);
  const std::int64_t slot_ns = calendar.slot_length().ns();
  if (!offset || offset->ns() < 0 || offset->ns() % slot_ns != 0) {
    return std::nullopt;
  }
  return offset->ns() / slot_ns;
}

}  // namespace

std::optional<ResourceSnapshot> open_window(const ServiceProfile& profile,
                                            const std::vector<WorkerSpec>& workers, TimePoint epoch,
                                            std::int64_t slot_count) {
  if (!valid_workers(workers)) {
    return std::nullopt;
  }
  std::optional<CapacityCalendar> uplink =
      calendar_at(profile.uplink, profile.slot_length, epoch, 0, slot_count);
  std::optional<CapacityCalendar> downlink =
      calendar_at(profile.downlink, profile.slot_length, epoch, 0, slot_count);
  std::optional<CapacityCalendar> medium;
  if (profile.medium) {
    medium = calendar_at(*profile.medium, profile.slot_length, epoch, 0, slot_count);
    if (!medium) {
      return std::nullopt;
    }
  }
  if (!uplink || !downlink) {
    return std::nullopt;
  }
  std::vector<Worker> lanes;
  lanes.reserve(workers.size());
  std::ranges::transform(workers, std::back_inserter(lanes), [](const WorkerSpec& worker) {
    return Worker{.id = worker.id, .profiles = worker.profiles, .lane = IntervalCalendar{}};
  });
  ResourceSnapshot snapshot{.uplink = std::move(*uplink),
                            .downlink = std::move(*downlink),
                            .workers = std::move(lanes),
                            .shared_medium = std::move(medium),
                            .uplink_delay = profile.uplink_delay,
                            .downlink_delay = profile.downlink_delay,
                            .dispatch_lead = profile.dispatch_lead,
                            .source = profile.source};
  if (!is_consistent(snapshot)) {
    return std::nullopt;
  }
  return snapshot;
}

std::optional<ResourceSnapshot> advance_window(const ResourceSnapshot& current,
                                               const ServiceProfile& profile, TimePoint epoch,
                                               TimePoint now) {
  if (current.uplink.slot_length() != profile.slot_length ||
      current.shared_medium.has_value() != profile.medium.has_value() ||
      !current.endpoints.empty()) {
    return std::nullopt;
  }
  const std::optional<std::int64_t> current_first = first_slot_of(current.uplink, epoch);
  const std::optional<Duration> since_epoch = elapsed(epoch, now);
  if (!current_first || !since_epoch) {
    return std::nullopt;
  }
  const std::int64_t target_first = since_epoch->ns() / profile.slot_length.ns();
  if (since_epoch->ns() < 0 || target_first <= *current_first) {
    return current;
  }
  ResourceSnapshot next = current;
  std::optional<CapacityCalendar> uplink =
      slid(current.uplink, profile.uplink, epoch, *current_first, target_first);
  std::optional<CapacityCalendar> downlink =
      slid(current.downlink, profile.downlink, epoch, *current_first, target_first);
  if (!uplink || !downlink) {
    return std::nullopt;
  }
  if (current.shared_medium && profile.medium) {
    next.shared_medium =
        slid(*current.shared_medium, *profile.medium, epoch, *current_first, target_first);
    if (!next.shared_medium) {
      return std::nullopt;
    }
  }
  next.uplink = std::move(*uplink);
  next.downlink = std::move(*downlink);
  const std::optional<TimePoint> origin = next.uplink.slot_start(SlotIndex{0});
  if (!origin) {
    return std::nullopt;
  }
  for (Worker& worker : next.workers) {
    worker.lane = worker.lane.pruned(*origin);
  }
  if (!is_consistent(next)) {
    return std::nullopt;
  }
  return next;
}

}  // namespace rovi::core
