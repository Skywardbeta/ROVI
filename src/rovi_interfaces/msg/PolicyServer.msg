# A policy server a worker accepted, published once per connection with transient-local durability.
# adapter_id is the SHA-256 of the server's adapter source; runtime names its libraries. connected_ns is the
# PC's ROS time of the request that opened the connection.

uint64 worker_id
uint32 connection
int64 connected_ns
string host
uint16 port
string policy_id
string adapter_id
string policy_type
string task
string robot_type
string device
string runtime
uint32 action_steps
uint32 action_dim
uint32 state_dim
uint32 image_width
uint32 image_height
int64 action_period_ns
