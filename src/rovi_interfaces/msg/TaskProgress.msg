# Pick-and-place progress of one arm, published by its controller.
# stamp_ns is ROS time in integer nanoseconds.

uint32 robot_id
int64 stamp_ns
uint32 goal_index
uint64 cycles_completed
bool moving
