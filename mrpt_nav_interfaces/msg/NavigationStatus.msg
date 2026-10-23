# Live status of a navigator (planner readiness and current goal), published
# whatever the source of the goal (topic or action).

std_msgs/Header header

uint8 STATE_INITIALIZING=0
uint8 STATE_INIT_FAILED=1
uint8 STATE_READY=2
uint8 STATE_PLANNING=3
uint8 STATE_FOLLOWING_PARTIAL=4
uint8 STATE_FOLLOWING=5
uint8 STATE_REPLANNING=6
uint8 STATE_BLOCKED=7
uint8 STATE_STALLED=8
uint8 STATE_SUCCEEDED=9
uint8 STATE_FAILED=10
uint8 STATE_CANCELED=11

uint8 state
string state_name        # e.g. "FOLLOWING", for people and text panels

# Severity, to color UIs: same values as diagnostic_msgs/DiagnosticStatus
uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ERROR=2
uint8 level

# Stable machine-readable reason of the latest notable event, empty if none:
#   ptgs_initializing, init_failed, goal_queued (INITIALIZING with a goal
#   waiting), goal_in_collision, invalid_goal, no_robot_pose, no_path,
#   path_published (one-shot planning only), no_progress, follower_blocked,
#   follower_failed, too_many_restarts, planning_error, replaced, canceled,
#   reached, waiting_partial_path, searching, robot_not_moving
string reason
string message           # human-readable, one line

# Initialization (valid in INITIALIZING):
float32 init_progress    # [0,1]
string init_detail       # e.g. "PTG 2/4: DiffDrive_C (building grid)"

# Current goal (goal_id == 0: none):
uint32 goal_id
bool goal_from_action
geometry_msgs/PoseStamped goal

float32 elapsed                 # [s] since the goal was received
float32 distance_to_goal        # [m] straight line (NaN if unknown)
float32 remaining_path_length   # [m] along the published path (NaN if none)
bool path_is_partial
float32 eta                     # [s] rough estimate along the path (NaN if unknown)
float32 time_since_progress     # [s]
float32 no_progress_timeout     # [s] (<=0: disabled)
uint16 replans                  # search restarts for this goal

# Follower (as last reported):
string follower_status
