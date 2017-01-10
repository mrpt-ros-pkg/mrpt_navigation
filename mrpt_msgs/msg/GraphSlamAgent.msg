# Connection-related properties for a graphSLAM agent in a multi-robot
# environment.
#
# Currently used in the mrpt_graphslam_2d ROS application.

std_msgs/String name
std_msgs/String hostname

# IPv4 of the corresponding agent
std_msgs/String ip_addr

# Port that the the roscore of the corresponding agent is running under
uint16 port

# True if the SLAM Agent was last reported to be online
std_msgs/Bool is_online

# Timestamp that the SLAM Agent was last seen
std_msgs/Time last_seen_time

# ROS Topics namespace that the agent is running under
# By definition this is going to be
# ${name}_${agent_ID} 
std_msgs/String topic_namespace

# this is the last field of the IPv4 address of the corresponding agent
# e.g. IP=192.168.100.17
# agent_ID = 17
int32 agent_ID 



