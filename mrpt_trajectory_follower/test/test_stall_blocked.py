# Integration test: the trajectory follower node reports Blocked when the
# robot doesn't move although commanded to, e.g. pushing against an obstacle
# it doesn't see (simulation with mvsim, headless).
#
# The robot (in aisles.world.xml, at the origin facing +x) is given a straight
# path through the column of static boxes ahead, while the obstacle cloud it
# gets is empty, so neither its predictive safety nor its collision guard
# slow it down: it drives into the boxes and stays there.
#
# Checks: the leg ends in Blocked (not in ReachedGoal, nor running forever),
# with the robot stopped in front of the boxes.
#
# Run it with:
#   colcon test --packages-select mrpt_trajectory_follower

import math
import os
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from launch_ros.actions import Node
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

TEST_DIR = os.path.dirname(os.path.abspath(__file__))

STATUS_TOPIC = '/mrpt_trajectory_follower_node/status'
PATH_TOPIC = '/waypoints_path'
OBSTACLES_TOPIC = '/test_obstacles'

BOXES_X = 4.0          # [m] near face of the column of boxes ahead
ROBOT_FRONT_X = 1.4    # [m] front of the footprint, from the robot origin
STARTUP_TIMEOUT = 60.0  # [s]
LEG_TIMEOUT = 60.0      # [s] driving there, plus the follower block_timeout


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mvsim = Node(
        package='mvsim', executable='mvsim_node', name='mvsim', output='screen',
        parameters=[{
            'world_file': os.path.join(TEST_DIR, 'aisles.world.xml'),
            'do_fake_localization': True,
            'headless': True,
        }])
    follower = Node(
        package='mrpt_trajectory_follower',
        executable='mrpt_trajectory_follower_node',
        name='mrpt_trajectory_follower_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id_map': 'map',
            'frame_id_robot': 'base_link',
            'ptg_ini': os.path.join(TEST_DIR, 'compact_car_shape.ini'),
            'follower_parameters': os.path.join(TEST_DIR, 'follower_params.yaml'),
            'topic_path_sub': PATH_TOPIC,
            'topic_obstacles_sub': OBSTACLES_TOPIC,
            'topic_odom_sub': '/odom',
            'topic_cmd_vel_pub': '/cmd_vel',
        }])
    return launch.LaunchDescription([
        mvsim, follower, launch_testing.actions.ReadyToTest()])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class StallBlocked(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'stall_blocked_test',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.status = None
        cls.status_count = 0
        cls.cmds = []

        def on_gt(m):
            p = m.pose.pose
            cls.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

        def on_status(m):
            cls.status = m.data
            cls.status_count += 1

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(Twist, '/cmd_vel', lambda m: cls.cmds.append(m.linear.x), 10)
        cls.pub_path = n.create_publisher(Path, PATH_TOPIC, latched)
        cls.pub_obs = n.create_publisher(PointCloud2, OBSTACLES_TOPIC, 10)
        n.create_timer(0.1, cls.publish_no_obstacles)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_no_obstacles(cls):
        # Fresh but empty: the guard sees nothing (and no stale data).
        h = Header()
        h.frame_id = 'map'
        h.stamp = cls.node.get_clock().now().to_msg()
        cls.pub_obs.publish(point_cloud2.create_cloud_xyz32(h, []))

    def test_blocked_when_not_moving(self):
        t0 = time.time()
        while self.gt is None or self.status is None:
            self.assertLess(time.time() - t0, STARTUP_TIMEOUT, 'Nodes did not start')
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Straight ahead, through the boxes:
        x0, y0, _ = self.gt
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(13):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x0 + 0.5 * i
            ps.pose.position.y = y0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        count0 = self.status_count
        self.pub_path.publish(msg)

        t0 = time.time()
        started = False
        while time.time() - t0 < LEG_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.status_count > count0 and self.status == 'Running':
                started = True
            if started and self.status not in ('Running', None):
                break
        x, _, _ = self.gt
        self.node.get_logger().info(
            f'status {self.status} after {time.time() - t0:.1f} s, robot x={x:.2f}')
        self.assertTrue(started, 'The follower never started')
        self.assertEqual(self.status, 'Blocked')
        self.assertLess(x + ROBOT_FRONT_X, BOXES_X + 0.05, 'The robot went through the boxes')

        # Once Blocked, it stops pushing:
        self.cmds.clear()
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(all(abs(v) < 1e-3 for v in self.cmds),
                        f'Still commanding motion while Blocked: {self.cmds[:5]}')
