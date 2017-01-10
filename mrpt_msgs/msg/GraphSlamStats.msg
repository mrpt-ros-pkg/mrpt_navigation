# Statistics related to the execution of graphSLAM.
# Message is utilized in the mrpt_graphslam ROS package

# Time of message acquisition
Header header

# node-related stats
int32 nodes_total

# edge-related stats
int32 edges_total
int32 edges_ICP2D
int32 edges_ICP3D
int32 edges_odom
int32 loop_closures

# Evaluation metric of the SLAM process
float64[] slam_evaluation_metric
