# Observation-to-action-chunk job submitted by a robot agent to the scheduler.
# Every *_ns time is ROS time (RCL_ROS_TIME) in integer nanoseconds; under use_sim_time it is simulation time.
# Durations are integer nanoseconds and sizes are integer bytes.
# Compatibility: fields are only appended within one RunMetadata.INTERFACE_VERSION.

uint64 job_id
uint32 robot_id
uint64 sequence

int64 release_time_ns
int64 observation_time_ns
int64 buffer_empty_time_ns
int64 safety_margin_ns

bool has_task_deadline
int64 task_deadline_ns

bool has_maximum_result_age
int64 maximum_result_age_ns

int64 upload_bytes
int64 action_chunk_bytes
uint32 model_class
uint64[] eligible_workers

uint64[] predecessors
bool has_synchronization_group
uint32 synchronization_group
int64 maximum_result_ready_skew_ns
# Members the scheduler waits for before planning the group; must be 0 for an ungrouped job.
uint32 synchronization_group_size
