# Modeled reservation of one admitted chain. These are planned times, not measured delivery.
# Times are ROS time in integer nanoseconds and sizes are integer bytes.

uint64 worker_id

int64 upload_bytes
int64 upload_start_ns
int64 upload_completion_ns
int64 observation_ready_ns

int64 compute_start_ns
int64 compute_end_ns

int64 download_bytes
int64 download_start_ns
int64 download_completion_ns
int64 delivery_ns

int64 deadline_ns
int64 slack_ns
