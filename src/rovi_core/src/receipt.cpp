#include "rovi_core/receipt.hpp"

#include <optional>
#include <utility>

#include "rovi_core/units.hpp"

namespace rovi::core {

std::optional<Fraction> ReceiptSummary::on_time_ratio() const {
  if (offered == 0 || pending != 0) {
    return std::nullopt;
  }
  return Fraction{.count = on_time, .of = offered};
}

std::optional<std::pair<Fraction, Fraction>> ReceiptSummary::on_time_bounds() const {
  if (offered == 0) {
    return std::nullopt;
  }
  return std::pair{Fraction{.count = on_time, .of = offered},
                   Fraction{.count = on_time + pending, .of = offered}};
}

// A receipt counts only if it was observed by the horizon, and absence is established only once
// the horizon is past the deadline, so a receipt exactly at the deadline is never pre-empted.
ReceiptOutcome classify_receipt(TimePoint deadline, std::optional<TimePoint> receipt,
                                TimePoint horizon) {
  if (receipt && *receipt <= horizon) {
    return *receipt <= deadline ? ReceiptOutcome::kOnTime : ReceiptOutcome::kLate;
  }
  return deadline < horizon ? ReceiptOutcome::kMissing : ReceiptOutcome::kPending;
}

bool ReceiptLedger::offer(JobId job, TimePoint observation, TimePoint deadline) {
  return open_
      .try_emplace(job,
                   Open{.observation = observation, .deadline = deadline, .worker = std::nullopt})
      .second;
}

bool ReceiptLedger::assign(JobId job, WorkerId worker) {
  const auto request = open_.find(job);
  if (request == open_.end() || request->second.worker) {
    return false;
  }
  request->second.worker = worker;
  return true;
}

std::optional<Receipt> ReceiptLedger::receive(const Response& response, TimePoint at) {
  const auto request = open_.find(response.job);
  if (request == open_.end() || request->second.worker != response.worker ||
      request->second.observation != response.observation) {
    return std::nullopt;
  }
  const TimePoint deadline = request->second.deadline;
  open_.erase(request);
  const ReceiptOutcome outcome = classify_receipt(deadline, at, at);
  ++(outcome == ReceiptOutcome::kOnTime ? on_time_ : late_);
  return Receipt{.job = response.job, .deadline = deadline, .at = at, .outcome = outcome};
}

std::optional<TimePoint> ReceiptLedger::deadline_of(JobId job) const {
  const auto request = open_.find(job);
  return request == open_.end() ? std::nullopt : std::optional{request->second.deadline};
}

ReceiptSummary ReceiptLedger::summary(TimePoint horizon) const {
  ReceiptSummary summary{
      .offered = on_time_ + late_ + open_.size(), .on_time = on_time_, .late = late_};
  for (const auto& [job, request] : open_) {
    ++(classify_receipt(request.deadline, std::nullopt, horizon) == ReceiptOutcome::kMissing
           ? summary.missing
           : summary.pending);
  }
  return summary;
}

}  // namespace rovi::core
