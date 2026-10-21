# Action chunk accepted by a robot agent, for its controller. Step k applies at start_ns + k * period_ns, and a
# newer chunk supersedes an older one from its own start_ns. An independent robot starts a chunk at its
# acceptance or when the chunk still playing ends, whichever is later. The agent's buffer always ends at the
# newest chunk's start_ns + steps * period_ns. Values are in policy units.

uint32 robot_id
uint64 job_id
string policy_id
int64 accepted_ns
int64 start_ns
int64 period_ns
uint32 steps
uint32 dim
float32[] actions
