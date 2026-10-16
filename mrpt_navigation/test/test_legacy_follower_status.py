# Integration test: the planner goal manager with the legacy follower status
# (plain std_msgs/String, without goal id and revision), selected by setting
# topic_follower_status_ex_sub to an empty string. A goal must still be
# planned, followed and reported as succeeded, end-to-end in simulation
# (mvsim, headless, diffdrive robot).
#
# Run it with:
#   colcon test --packages-select mrpt_navigation

import math
import os
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mrpt_nav_interfaces.action import NavigateGoal
from mrpt_nav_interfaces.msg import NavigationFinalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

STATUS_TOPIC = '/mrpt_trajectory_follower/status'
ROBOT_SHAPE_TOPIC = '/mrpt_tps_astar_planner_node/robot_shape'
ACTION_NAME = '/mrpt_tps_astar_planner_node/navigate_goal'

STARTUP_TIMEOUT = 300.0  # [s]
GOAL_TIMEOUT = 150.0  # [s]
GOAL = (9.20, 17.95, -165.0)  # x [m], y [m], yaw [deg]; the robot starts at (30, 30)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    demo = os.path.join(
        get_package_share_directory('mrpt_tutorials'), 'launch',
        'demo_astar_trajectory_follower_gridmap.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo),
            launch_arguments={
                'robot': 'diffdrive',
                'speed_limit': '1.0',
                'use_rviz': 'False',
                'topic_follower_status_ex_sub': '',
                'pf_params_overrides_file': os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 'pf-test-overrides.yaml'),
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestLegacyFollowerStatus(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('legacy_follower_status_test')
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.status = None
        cls.collisions = 0
        cls.robot_shape = None

        def on_gt(m):
            p = m.pose.pose
            cls.gt = (p.position.x, p.position.y)

        def on_status(m):
            cls.status = m.data

        def on_collision(m):
            if m.data:
                cls.collisions += 1

        def on_shape(m):
            cls.robot_shape = [(p.x, p.y) for p in m.polygon.points]

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        n.create_subscription(PolygonStamped, ROBOT_SHAPE_TOPIC, on_shape, latched)
        cls.action = ActionClient(n, NavigateGoal, ACTION_NAME)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_goal_succeeds_with_the_plain_status(self):
        ok = self.spin_until(
            lambda: self.robot_shape is not None and self.gt is not None and
            self.status is not None and self.action.server_is_ready(), STARTUP_TIMEOUT)
        self.assertTrue(ok, 'navigation nodes not ready')
        self.spin_until(lambda: False, 10.0)  # let the localization converge

        goal = NavigateGoal.Goal()
        goal.target = PoseStamped()
        goal.target.header.frame_id = 'map'
        goal.target.header.stamp = self.node.get_clock().now().to_msg()
        goal.target.pose.position.x = GOAL[0]
        goal.target.pose.position.y = GOAL[1]
        yaw = math.radians(GOAL[2])
        goal.target.pose.orientation.z = math.sin(yaw / 2)
        goal.target.pose.orientation.w = math.cos(yaw / 2)
        fut = self.action.send_goal_async(goal)
        self.assertTrue(self.spin_until(fut.done, 10.0), 'goal not answered')
        handle = fut.result()
        self.assertTrue(handle.accepted, 'goal rejected')

        result_fut = handle.get_result_async()
        self.assertTrue(self.spin_until(result_fut.done, GOAL_TIMEOUT),
                        'the goal did not end in time')
        res = result_fut.result()
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(res.result.state.navigation_status,
                         NavigationFinalStatus.STATUS_SUCCESS)
        self.assertLess(math.hypot(self.gt[0] - GOAL[0], self.gt[1] - GOAL[1]), 0.5)
        self.assertEqual(self.collisions, 0)
