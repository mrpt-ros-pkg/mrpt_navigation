# NodeID along with its corresponding 2D/3D pose estimation.
# Message also contains the following optional fields:
# - str_ID: string identifier of the robot agent that initially registered this node in
# its graph
# - nodeID_loc: ID of the current node in the graph it was initially registered in.
#
# note: Additional fields correspond to the fields of the
# mrpt::graphs::detail::TMRSlamNodeAnnotations struct
#
# Currently used in the mrpt_graphslam_2d ROS application.

uint64 nodeID
geometry_msgs/Pose pose

std_msgs/String str_ID
uint64 nodeID_loc
