# A reference path for a path follower, from a planner that keeps updating it
# while the robot drives toward its goal.

# The path (frame and poses):
nav_msgs/Path path

# Identifies the navigation goal: paths with the same goal_id continue each
# other (the follower keeps its motion state when switching between them).
uint32 goal_id

# Increases with each update published for the same goal.
uint32 revision

# True if the path does not reach the goal yet: more updates will follow,
# so the follower must stop at its end, if reached, and wait for them instead
# of reporting the goal as reached.
bool is_partial
