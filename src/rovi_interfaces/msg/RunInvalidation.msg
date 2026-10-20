# A component stopped because its ROS time moved backward. Its outstanding work is discarded, it
# processes nothing more, and the run must be restarted cleanly. Published once, transient-local.

string node
int64 latest_ns
int64 observed_ns
