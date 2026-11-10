# Integration test: the trajectory follower node stops the robot when its
# obstacle data stop arriving (e.g. a stalled LiDAR or obstacle pipeline), and
# resumes once they are back (simulation with mvsim, headless).
#
# The robot (in aisles.world.xml, at the origin facing +x) follows a short
# straight path with fresh but empty obstacle clouds. While it drives, the
# clouds stop for a while.
#
# Checks: within the collision guard max_obstacles_age (plus a control period)
# all commands are zero and the robot stops; with clouds again, it drives on.
# Times are measured on the simulation clock, which may run slower than real
# time on a loaded machine.
#
# Run it with:
#   colcon test --packages-select mrpt_trajectory_follower

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

MAX_OBSTACLES_AGE = 0.3  # [s] as in follower_params_stale_obstacles.yaml
STOP_DELAY = MAX_OBSTACLES_AGE + 0.2  # [s] plus control period and latencies
STARTUP_TIMEOUT = 60.0  # [s]
DRIVE_TIMEOUT = 20.0    # [s]


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
            'follower_parameters_overrides': os.path.join(
                TEST_DIR, 'follower_params_stale_obstacles.yaml'),
        }])
    return launch.LaunchDescription([
        mvsim, follower, launch_testing.actions.ReadyToTest()])


class StaleObstaclesStop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'stale_obstacles_stop_test',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.gt_speed = None
        cls.status = None
        cls.cmds = []  # (sim time [s], v)
        cls.obstacles_on = True

        def on_gt(m):
            p = m.pose.pose
            cls.gt = (p.position.x, p.position.y)
            cls.gt_speed = abs(m.twist.twist.linear.x)

        def on_status(m):
            cls.status = m.data

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(
            Twist, '/cmd_vel', lambda m: cls.cmds.append((cls.sim_now(), m.linear.x)), 10)
        cls.pub_path = n.create_publisher(Path, PATH_TOPIC, latched)
        cls.pub_obs = n.create_publisher(PointCloud2, OBSTACLES_TOPIC, 10)
        n.create_timer(0.1, cls.publish_no_obstacles)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def sim_now(cls):
        return cls.node.get_clock().now().nanoseconds * 1e-9

    @classmethod
    def publish_no_obstacles(cls):
        if not cls.obstacles_on:
            return
        h = Header()
        h.frame_id = 'map'
        h.stamp = cls.node.get_clock().now().to_msg()
        cls.pub_obs.publish(point_cloud2.create_cloud_xyz32(h, []))

    def spin_sim_for(self, seconds):
        t_end = self.sim_now() + seconds
        t0 = time.time()
        while self.sim_now() < t_end:
            self.assertLess(time.time() - t0, 10 * seconds + 10, 'The simulation clock stalled')
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_until_driving(self, what):
        t0 = time.time()
        while not (self.cmds and self.cmds[-1][1] > 0.1):
            self.assertLess(time.time() - t0, DRIVE_TIMEOUT, f'The robot never {what}')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_stops_on_stale_obstacles(self):
        t0 = time.time()
        while self.gt is None or self.status is None:
            self.assertLess(time.time() - t0, STARTUP_TIMEOUT, 'Nodes did not start')
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Straight ahead, stopping well before the boxes:
        x0, y0 = self.gt
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(5):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x0 + 0.5 * i
            ps.pose.position.y = y0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)
        self.wait_until_driving('started')

        # The obstacle data stop:
        self.__class__.obstacles_on = False
        t_stall = self.sim_now()
        self.spin_sim_for(STOP_DELAY + 1.0)
        late = [v for t, v in self.cmds if t > t_stall + STOP_DELAY]
        self.assertTrue(late, 'No commands while the obstacle data were stale')
        self.assertTrue(all(abs(v) < 1e-3 for v in late),
                        f'Still commanding motion with stale obstacles: {late[:5]}')
        t0 = time.time()
        while self.gt_speed >= 0.02:
            self.assertLess(time.time() - t0, 5.0, 'The robot did not stop')
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Fresh data again: it drives on.
        self.__class__.obstacles_on = True
        self.wait_until_driving('resumed')
