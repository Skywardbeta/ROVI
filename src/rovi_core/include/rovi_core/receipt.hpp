// Receipt outcomes of offered requests against their original deadlines, independent of group
// release and action acceptance, classified at an observation horizon.
#ifndef ROVI_CORE_RECEIPT_HPP_
#define ROVI_CORE_RECEIPT_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <utility>

#include "rovi_core/units.hpp"

namespace rovi::core {

enum class ReceiptOutcome : std::uint8_t { kOnTime, kLate, kMissing, kPending };

struct Fraction {
  std::uint64_t count{0};
  std::uint64_t of{0};

  bool operator==(const Fraction&) const = default;
};

struct ReceiptSummary {
  std::uint64_t offered{0};
  std::uint64_t on_time{0};
  std::uint64_t late{0};
  std::uint64_t missing{0};
  std::uint64_t pending{0};

  [[nodiscard]] std::optional<Fraction> on_time_ratio() const;
  [[nodiscard]] std::optional<std::pair<Fraction, Fraction>> on_time_bounds() const;

  bool operator==(const ReceiptSummary&) const = default;
};

struct Response {
  JobId job;
  WorkerId worker;
  TimePoint observation;
};

struct Receipt {
  JobId job;
  TimePoint deadline;
  TimePoint at;
  ReceiptOutcome outcome{ReceiptOutcome::kOnTime};

  bool operator==(const Receipt&) const = default;
};

[[nodiscard]] ReceiptOutcome classify_receipt(TimePoint deadline, std::optional<TimePoint> receipt,
                                              TimePoint horizon);

// A response counts only if it matches an offered request's job, observation, and admitted
// worker. A received request is folded into the counters and forgotten, so memory and summary
// cost grow only with unanswered requests; summary() therefore assumes a horizon no earlier than
// any recorded receipt, which holds when the horizon is the current time.
class ReceiptLedger {
 public:
  bool offer(JobId job, TimePoint observation, TimePoint deadline);
  bool assign(JobId job, WorkerId worker);
  std::optional<Receipt> receive(const Response& response, TimePoint at);
  [[nodiscard]] std::optional<TimePoint> deadline_of(JobId job) const;
  [[nodiscard]] ReceiptSummary summary(TimePoint horizon) const;

 private:
  struct Open {
    TimePoint observation;
    TimePoint deadline;
    std::optional<WorkerId> worker;
  };

  std::map<JobId, Open> open_;
  std::uint64_t on_time_{0};
  std::uint64_t late_{0};
};

}  // namespace rovi::core

#endif  // ROVI_CORE_RECEIPT_HPP_
