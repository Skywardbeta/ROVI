// Pending, decided, and completed job bookkeeping between scheduler cycles.
#ifndef ROVI_SCHEDULER_JOB_BOOK_HPP_
#define ROVI_SCHEDULER_JOB_BOOK_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "rovi_core/admission.hpp"
#include "rovi_core/job.hpp"
#include "rovi_core/scheduler.hpp"
#include "rovi_core/units.hpp"

namespace rovi::scheduler {

inline constexpr std::size_t kMaximumRequestListLength = 64;
inline constexpr std::size_t kClosedGroupMemory = 4'096;

struct BookLimits {
  core::Duration release_tolerance;
  std::size_t max_pending{0};
};

struct Notice {
  core::Job job;
  core::JobDecision decision;

  bool operator==(const Notice&) const = default;
};

class JobBook {
 public:
  explicit JobBook(BookLimits limits);

  [[nodiscard]] std::optional<core::JobDecision> submit(const core::Job& job, core::TimePoint now,
                                                        std::uint32_t group_size = 0);
  bool complete(core::JobId job);
  [[nodiscard]] std::vector<core::Job> released(core::TimePoint now) const;
  [[nodiscard]] std::vector<core::JobId> completed_for(const std::vector<core::Job>& jobs) const;
  [[nodiscard]] std::size_t pending_count() const;
  [[nodiscard]] std::vector<Notice> expire(core::TimePoint now);
  [[nodiscard]] std::vector<Notice> settle(const std::vector<core::JobDecision>& decisions);

 private:
  [[nodiscard]] std::optional<core::ReasonCode> refusal(const core::Job& job, core::TimePoint now,
                                                        std::uint32_t group_size) const;
  [[nodiscard]] std::optional<core::ReasonCode> group_refusal(const core::Job& job,
                                                              std::uint32_t group_size) const;
  [[nodiscard]] bool group_complete(core::GroupId group) const;
  [[nodiscard]] bool group_ready(const core::Job& job, core::TimePoint now) const;
  void close_group(const core::Job& job);

  BookLimits limits_;
  std::vector<core::Job> pending_;
  std::set<core::JobId> decided_;
  std::set<core::JobId> admitted_;
  std::map<core::JobId, core::ReasonCode> reported_deferrals_;
  std::set<core::JobId> completed_;
  std::map<core::GroupId, std::uint32_t> group_sizes_;
  std::set<core::GroupId> closed_groups_;
  std::deque<core::GroupId> closed_order_;
};

}  // namespace rovi::scheduler

#endif  // ROVI_SCHEDULER_JOB_BOOK_HPP_
