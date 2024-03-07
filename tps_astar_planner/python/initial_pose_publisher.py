#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

def pose_callback(pose):
    print(pose)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('initial_pose_node')

    # Get the initial position parameters
    initial_x   = rospy.get_param('initial_x', -2.0)
    initial_y   = rospy.get_param('initial_y', 3.0)
    initial_yaw = rospy.get_param('initial_phi', 1.57)

    # Create a PoseWithCovarianceStamped message
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.pose.position.x = initial_x
    initial_pose.pose.pose.position.y = initial_y
    q = quaternion_from_euler(0,0,initial_yaw)
    initial_pose.pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    # Publish the initial pose to the /initialpose topic
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    sub = rospy.Subscriber('mrpt_pose', PoseWithCovarianceStamped, pose_callback)
    rate = rospy.Rate(1)  # 10 Hz
    while not rospy.is_shutdown():
        pub.publish(initial_pose)
        rate.sleep()
        break
