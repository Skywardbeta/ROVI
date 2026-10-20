// GoogleTest output operators that render rovi_core value types readably in assertion failures.
#ifndef ROVI_CORE_TEST_PRINTERS_HPP_
#define ROVI_CORE_TEST_PRINTERS_HPP_

#include <concepts>
#include <ostream>

#include "rovi_core/admission.hpp"

namespace rovi::core {

template <typename Tag, std::integral Rep>
std::ostream& operator<<(std::ostream& os, const StrongValue<Tag, Rep>& value) {
  return os << value.value();
}

inline std::ostream& operator<<(std::ostream& os, Duration value) {
  return os << value.ns() << "ns";
}

inline std::ostream& operator<<(std::ostream& os, TimePoint value) {
  return os << "t=" << value.ns() << "ns";
}

inline std::ostream& operator<<(std::ostream& os, const ServiceSlice& value) {
  return os << "{slot " << value.slot << ", " << value.bytes << "B}";
}

inline std::ostream& operator<<(std::ostream& os, const Transfer& value) {
  os << "{slices [";
  for (const ServiceSlice& slice : value.slices) {
    os << slice << ' ';
  }
  return os << "], completion " << value.completion << "}";
}

inline std::ostream& operator<<(std::ostream& os, const Interval& value) {
  return os << "[" << value.start << ", " << value.end << ")";
}

inline std::ostream& operator<<(std::ostream& os, DecisionStatus value) {
  switch (value) {
    case DecisionStatus::kAdmitted:
      return os << "Admitted";
    case DecisionStatus::kDeferred:
      return os << "Deferred";
    case DecisionStatus::kRejected:
      return os << "Rejected";
  }
  return os << "DecisionStatus(" << static_cast<int>(value) << ")";
}

inline std::ostream& operator<<(std::ostream& os, ReasonCode value) {
  switch (value) {
    case ReasonCode::kNone:
      return os << "None";
    case ReasonCode::kInvalidSnapshot:
      return os << "InvalidSnapshot";
    case ReasonCode::kInvalidPayloadSize:
      return os << "InvalidPayloadSize";
    case ReasonCode::kInvalidTiming:
      return os << "InvalidTiming";
    case ReasonCode::kDuplicateJob:
      return os << "DuplicateJob";
    case ReasonCode::kArithmeticOverflow:
      return os << "ArithmeticOverflow";
    case ReasonCode::kNoEligibleWorker:
      return os << "NoEligibleWorker";
    case ReasonCode::kPredecessorIncomplete:
      return os << "PredecessorIncomplete";
    case ReasonCode::kUplinkServiceUnavailable:
      return os << "UplinkServiceUnavailable";
    case ReasonCode::kNoFeasibleChain:
      return os << "NoFeasibleChain";
    case ReasonCode::kDeadlineMiss:
      return os << "DeadlineMiss";
    case ReasonCode::kStaleResult:
      return os << "StaleResult";
    case ReasonCode::kGroupMemberUnavailable:
      return os << "GroupMemberUnavailable";
    case ReasonCode::kGroupSkewExceeded:
      return os << "GroupSkewExceeded";
    case ReasonCode::kQueueFull:
      return os << "QueueFull";
    case ReasonCode::kMalformedRequest:
      return os << "MalformedRequest";
  }
  return os << "ReasonCode(" << static_cast<int>(value) << ")";
}

inline std::ostream& operator<<(std::ostream& os, const Decision& value) {
  return os << value.status << "/" << value.reason;
}

}  // namespace rovi::core

#endif  // ROVI_CORE_TEST_PRINTERS_HPP_
