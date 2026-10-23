int8 navigation_status

int8 STATUS_SUCCESS=0
int8 STATUS_ERROR=1
int8 STATUS_CANCELLED=2

# Optional details of how the navigation ended (empty if not provided).
# `reason` is a stable machine-readable code (see NavigationStatus.msg),
# `message` a one-line human-readable description:
string reason
string message
