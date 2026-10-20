// Logical end-to-end link that executes immutable reservations on ROS time: a payload ready by its
// lease's service start is released at the reserved arrival, and any other payload is refused.
#ifndef ROVI_NETWORK_LOGICAL_LINK_HPP_
#define ROVI_NETWORK_LOGICAL_LINK_HPP_

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "rovi_core/units.hpp"

namespace rovi::network {

enum class Direction : std::uint8_t { kUplink, kDownlink };

enum class Violation : std::uint8_t {
  kLateSend,
  kSizeMismatch,
  kIdentityMismatch,
  kDuplicate,
  kUnleased,
  kWrongDirection,
};

struct LinkTiming {
  core::Duration orphan_timeout;
  core::Duration max_lead;
};

struct Authorization {
  core::RobotId robot;
  core::WorkerId worker;
  core::Bytes bytes;

  bool operator==(const Authorization&) const = default;
};

struct Lease {
  core::TimePoint service_start;
  core::TimePoint arrival;
  Authorization authorized;

  bool operator==(const Lease&) const = default;
};

struct Refusal {
  core::JobId job;
  Direction direction{Direction::kUplink};
  Violation reason{Violation::kLateSend};
  Authorization claimed;
  core::TimePoint sent;
  core::TimePoint detected;
  std::optional<Lease> lease;

  bool operator==(const Refusal&) const = default;
};

[[nodiscard]] inline std::optional<core::TimePoint> release_time(const Lease& lease,
                                                                 core::TimePoint sent) {
  if (lease.arrival < lease.service_start || sent > lease.service_start) {
    return std::nullopt;
  }
  return lease.arrival;
}

template <typename Payload>
class LogicalLink {
 public:
  struct Delivery {
    Direction direction{Direction::kUplink};
    Payload payload;
    core::TimePoint release;
  };

  struct Events {
    std::vector<Delivery> deliveries;
    std::vector<Refusal> refusals;
  };

  explicit LogicalLink(LinkTiming timing) : timing_{timing} {}

  bool lease(core::JobId job, Direction direction, const Lease& lease, core::TimePoint now) {
    const Key key{job, direction};
    const std::optional<core::TimePoint> latest_arrival = core::checked_add(now, timing_.max_lead);
    const bool beyond_lead = latest_arrival && lease.arrival > *latest_arrival;
    if (beyond_lead || lease.arrival < lease.service_start || lease.authorized.bytes.value() <= 0 ||
        leases_.contains(key) || settled_.contains(key)) {
      return false;
    }
    std::vector<Sent> candidates;
    if (const auto waiting = waiting_.find(key); waiting != waiting_.end()) {
      candidates = std::move(waiting->second);
      waiting_.erase(waiting);
    }
    bool consumed = false;
    for (Sent& sent : candidates) {
      if (consumed) {
        refuse(key, mismatch_of(lease, sent).value_or(Violation::kDuplicate), sent, lease, now);
      } else {
        consumed = offer(key, lease, std::move(sent), now);
      }
    }
    if (!consumed) {
      leases_.emplace(key, lease);
    }
    return true;
  }

  void send(core::JobId job, Direction direction, const Authorization& claimed, Payload payload,
            core::TimePoint now) {
    const Key key{job, direction};
    Sent sent{.claimed = claimed, .at = now, .payload = std::move(payload)};
    if (settled_.contains(key)) {
      refuse(key, Violation::kDuplicate, sent, std::nullopt, now);
      return;
    }
    const auto leased = leases_.find(key);
    if (leased == leases_.end()) {
      waiting_[key].push_back(std::move(sent));
      return;
    }
    if (offer(key, leased->second, std::move(sent), now)) {
      leases_.erase(leased);
    }
  }

  Events deliver(core::TimePoint now) {
    Events events;
    const auto first_pending = queue_.upper_bound(now);
    for (auto entry = queue_.begin(); entry != first_pending; ++entry) {
      events.deliveries.push_back(std::move(entry->second));
    }
    queue_.erase(queue_.begin(), first_pending);
    unused_leases_ += std::erase_if(leases_, [this, now](const auto& entry) {
      return core::elapsed_beyond(entry.second.arrival, timing_.orphan_timeout, now);
    });
    for (auto entry = waiting_.begin(); entry != waiting_.end();) {
      std::erase_if(entry->second, [this, &entry, now](const Sent& sent) {
        const bool orphaned = core::elapsed_beyond(sent.at, timing_.orphan_timeout, now);
        if (orphaned) {
          refuse(entry->first, Violation::kUnleased, sent, std::nullopt, now);
        }
        return orphaned;
      });
      entry = entry->second.empty() ? waiting_.erase(entry) : std::next(entry);
    }
    std::erase_if(settled_, [this, now](const auto& entry) {
      return core::elapsed_beyond(entry.second, timing_.orphan_timeout, now);
    });
    events.refusals = std::exchange(refused_, {});
    return events;
  }

  [[nodiscard]] std::uint64_t unused_leases() const { return unused_leases_; }

 private:
  using Key = std::pair<core::JobId, Direction>;

  struct Sent {
    Authorization claimed;
    core::TimePoint at;
    Payload payload;
  };

  static std::optional<Violation> mismatch_of(const Lease& lease, const Sent& sent) {
    if (sent.claimed.robot != lease.authorized.robot ||
        sent.claimed.worker != lease.authorized.worker) {
      return Violation::kIdentityMismatch;
    }
    if (sent.claimed.bytes != lease.authorized.bytes) {
      return Violation::kSizeMismatch;
    }
    return std::nullopt;
  }

  // A mismatched payload is refused without consuming the lease, so it cannot take the valid
  // payload's authorization. A matching payload consumes it even when late, and the key stays
  // settled for an orphan timeout past its arrival so nothing can obtain a second delivery.
  bool offer(const Key& key, const Lease& lease, Sent sent, core::TimePoint now) {
    if (const std::optional<Violation> mismatch = mismatch_of(lease, sent)) {
      refuse(key, *mismatch, sent, lease, now);
      return false;
    }
    settled_.emplace(key, std::max(lease.arrival, now));
    if (!release_time(lease, sent.at)) {
      refuse(key, Violation::kLateSend, sent, lease, now);
      return true;
    }
    queue_.emplace(lease.arrival, Delivery{.direction = key.second,
                                           .payload = std::move(sent.payload),
                                           .release = lease.arrival});
    return true;
  }

  void refuse(const Key& key, Violation reason, const Sent& sent, std::optional<Lease> lease,
              core::TimePoint detected) {
    refused_.push_back(Refusal{.job = key.first,
                               .direction = key.second,
                               .reason = reason,
                               .claimed = sent.claimed,
                               .sent = sent.at,
                               .detected = detected,
                               .lease = std::move(lease)});
  }

  LinkTiming timing_;
  std::map<Key, Lease> leases_;
  std::map<Key, std::vector<Sent>> waiting_;
  std::map<Key, core::TimePoint> settled_;
  std::multimap<core::TimePoint, Delivery> queue_;
  std::vector<Refusal> refused_;
  std::uint64_t unused_leases_{0};
};

}  // namespace rovi::network

#endif  // ROVI_NETWORK_LOGICAL_LINK_HPP_
