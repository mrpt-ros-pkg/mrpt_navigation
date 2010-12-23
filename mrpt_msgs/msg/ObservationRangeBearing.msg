# This message holds an observation of one or more landmarks 
#  detected by their range (distance) + bearing (2 or 3 angles) 
#  from some sensor on the robot.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# The sensor pose on the robot coordinates frame.
geometry_msgs/Pose sensor_pose_on_robot

# Sensor characteristics:
float64 min_sensor_distance   # In meters (m)
float64 max_sensor_distance   # (m)

# Typical sensor noise:
float64 sensor_std_range      # (m)
float64 sensor_std_yaw        # (rad)
float64 sensor_std_pitch      # (pitch)

# The vector of individual observations
SingleRangeBearingObservation[] sensed_data


