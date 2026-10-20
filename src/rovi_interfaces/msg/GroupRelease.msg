# Release or refusal of one cooperative group's action chunks, published by the coordinator.
# Times are ROS time in integer nanoseconds; reason values mirror Decision.REASON_*.

uint32 workcell
uint32 synchronization_group
bool released
uint8 reason
int64 release_time_ns
int64 result_ready_skew_ns
uint64[] jobs
uint32[] robots
