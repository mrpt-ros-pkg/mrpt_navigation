cmake_minimum_required(VERSION 2.8.3)
project(mrpt_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  ObservationRangeBearing.msg
  ObservationRangeBeacon.msg
  SingleRangeBearingObservation.msg
  SingleRangeBeaconObservation.msg
  GraphSlamStats.msg
  GraphSlamAgent.msg
  GraphSlamAgents.msg
  NodeIDWithPose.msg
  GraphConstraint.msg
  NetworkOfPoses.msg
  )


 add_service_files(
   DIRECTORY srv
   FILES
   GetCMGraph.srv
   )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
