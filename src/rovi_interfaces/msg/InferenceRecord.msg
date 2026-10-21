# One real inference by a policy worker. *_ns times are the PC's ROS time; server durations come from the
# inference host's own monotonic clock. planned_* is the reserved compute interval.

uint8 OUTCOME_OK=0
uint8 OUTCOME_UNREACHABLE=1
uint8 OUTCOME_TIMEOUT=2
uint8 OUTCOME_PROTOCOL=3
uint8 OUTCOME_IDENTITY=4
uint8 OUTCOME_REMOTE_ERROR=5
uint8 OUTCOME_INVALID_CHUNK=6

uint64 job_id
uint32 robot_id
uint64 worker_id
string policy_id
uint8 outcome
int64 planned_start_ns
int64 planned_end_ns
int64 request_ns
int64 response_ns
int64 round_trip_ns
int64 server_inference_ns
int64 server_handling_ns
uint64 noise_seed
# The worker's connection that served this inference, as announced on PolicyServer; 0 if none.
uint32 connection
