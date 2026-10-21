# Action-buffer, job, and fallback state of one robot agent.

uint8 MODE_EXECUTING=0
uint8 MODE_FALLBACK=1

uint32 robot_id
int64 stamp_ns
uint8 mode
int64 buffer_empty_time_ns
int64 buffer_remaining_ns
bool has_job_in_flight
uint64 job_in_flight
uint64 jobs_submitted
uint64 jobs_rejected
uint64 chunks_accepted
uint64 chunks_late
uint64 chunks_stale
uint64 chunks_missed
uint64 fallback_entries
# Receipt outcomes of this robot's offered requests against their original deadlines at stamp_ns,
# independent of group release and chunk acceptance (rovi_core/receipt.hpp).
uint64 requests_offered
uint64 receipts_on_time
uint64 receipts_late
uint64 requests_missing
uint64 requests_pending
# Delivered chunks whose content failed validation and were discarded.
uint64 chunks_invalid
