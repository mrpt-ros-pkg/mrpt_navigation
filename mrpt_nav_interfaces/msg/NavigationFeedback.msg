int16 total_waypoints
int16 reached_waypoints

# Optional, navigator-specific details (0 / empty if not provided):
# Straight-line distance from the robot to the goal [m]:
float32 distance_to_goal
# Current navigator state, e.g. "Planning", "Running", "Replanning":
string status
# Remaining length of the path being followed [m] (NaN if not provided):
float32 remaining_path_length
# Rough estimate of the time to reach the goal [s] (NaN if not provided):
float32 eta
# Time without getting closer to the goal [s] (0 if not provided):
float32 time_since_progress
