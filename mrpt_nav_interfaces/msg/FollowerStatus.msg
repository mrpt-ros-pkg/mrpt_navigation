# Status of a path follower, with the path it refers to, so the planner that
# published the path does not mistake the status of a previous path for the
# current one. The plain std_msgs/String status topic is kept for
# compatibility.

std_msgs/Header header

# One of: Idle, Running, WaitingForPath, ReachedGoal, Blocked, OffPathExceeded,
# MissedGoal, Failed, ...
string status

# True if the path being followed is a ReferencePath (below fields are valid).
bool has_goal_id

# goal_id and revision of the last ReferencePath adopted by the follower.
uint32 goal_id
uint32 revision
