int16 total_waypoints
int16 reached_waypoints

# Optional, navigator-specific details (0 / empty if not provided):
# Straight-line distance from the robot to the goal [m]:
float32 distance_to_goal
# Current navigator state, e.g. "Planning", "Running", "Replanning":
string status
