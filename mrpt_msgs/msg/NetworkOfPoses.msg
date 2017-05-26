# Represents a 2D/3D Directed Graph of Constraints
#
# Graph consists of:
# - NodeIDs with their corresponding poses
# - Directed Constraints that connect 2 nodes with each other
# - Root node marking the start of the graph

uint64 root
NodeIDWithPose_vec nodes
GraphConstraint[] constraints
