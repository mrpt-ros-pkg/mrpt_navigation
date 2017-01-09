# Represents a 3D Directed Graph of Constraints
# Currently used in the mrpt_graphslam_2d ROS application.
#
# Graph consists of
# - NodeIDs with their corresponding poses
# - Directed Constraints that connect 2 nodes with each other
# - Root node marking the start of the graph

uint64 root
NodeIDWithPose[] nodes
GraphConstraint[] constraints


