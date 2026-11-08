# Integration tests of waypoint sequences (planner `navigate_waypoints`
# action), end-to-end in simulation (mvsim, headless, diffdrive robot):
#  - A sequence of waypoints with free headings is driven through as one goal
#    (one goal id, waypoint counters in the status and feedback), passing
#    close to each waypoint without stopping at the intermediate ones, and
#    ends SUCCEEDED at the last one.
#  - An intermediate waypoint in collision that allows skipping is skipped,
#    and the sequence still succeeds.
#  - An obstacle source with no points (e.g. nothing in sight) does not stop
#    the planner from handling new goals.
#
# Run it with:
#   colcon test --packages-select mrpt_navigation

import math
import os
import time
import unittest

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mrpt_msgs.msg import Waypoint
from mrpt_nav_interfaces.action import NavigateWaypoints
from mrpt_nav_interfaces.msg import FollowerStatus, NavigationFinalStatus, NavigationStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformListener

PLANNER_NS = '/mrpt_tps_astar_planner_node'
STATUS_TOPIC = PLANNER_NS + '/navigation_status'
ACTION_NAME = PLANNER_NS + '/navigate_waypoints'
FOLLOWER_STATUS_TOPIC = '/mrpt_trajectory_follower/status_ex'
VIRTUAL_OBSTACLES_TOPIC = '/test_virtual_obstacles'

STARTUP_TIMEOUT = 300.0     # [s] nodes ready
SEQUENCE_TIMEOUT = 240.0    # [s] per sequence

# Waypoints (x, y) [m], map frame, with free headings. The robot starts at
# (30, 30): a route with gentle turns, through open areas, to a goal of the
# navigation battery.
ROUTE = [(24.0, 22.0), (15.0, 20.0), (9.20, 17.95)]
# Back, with the first waypoint in collision (and skippable):
ROUTE_BACK = [(15.0, 20.0), (24.0, 22.0)]
# The robot start pose:
HOME = (30.0, 30.0)

# Ground truth distance [m] from each waypoint accepted as passing through it:
WAYPOINT_GT_TOLERANCE = 1.5
# The robot must keep moving this fast [m/s] while this close [m] to the first
# intermediate waypoint (it would stop there if each waypoint were a goal):
MIN_SPEED_AT_WAYPOINT = 0.1
NEAR_WAYPOINT = 1.0


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
                'planner_extra_obstacles_topic': VIRTUAL_OBSTACLES_TOPIC,
                'pf_params_overrides_file': os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 'pf-test-overrides.yaml'),
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def make_waypoints(points, allow_skip=False):
    goal = NavigateWaypoints.Goal()
    goal.waypoints.header.frame_id = 'map'
    for x, y in points:
        wp = Waypoint()
        wp.target.position.x = x
        wp.target.position.y = y
        wp.target.orientation.w = 1.0
        wp.ignore_heading = True
        wp.allow_skip = allow_skip
        wp.allowed_distance = 0.0  # planner default
        goal.waypoints.waypoints.append(wp)
    return goal


class TestWaypoints(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('waypoints_test')
        n = cls.node
        latched_all = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.track = []              # (t, x, y) ground truth
        cls.collisions = 0
        cls.last = None             # latest NavigationStatus
        cls.messages = []
        cls.follower = []           # (t, FollowerStatus)
        cls.obstacles = None
        cls.last_obstacle_pub = 0.0

        def on_gt(m):
            p = m.pose.pose
            cls.gt = (p.position.x, p.position.y)
            cls.track.append((time.time(), p.position.x, p.position.y))

        def on_status(m):
            cls.last = m
            cls.messages.append(m)

        def on_follower(m):
            cls.follower.append((time.time(), m))

        def on_collision(m):
            if m.data:
                cls.collisions += 1

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(NavigationStatus, STATUS_TOPIC, on_status, latched_all)
        n.create_subscription(FollowerStatus, FOLLOWER_STATUS_TOPIC, on_follower, 100)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        cls.pub_obstacles = n.create_publisher(PointCloud2, VIRTUAL_OBSTACLES_TOPIC, 10)
        cls.action = ActionClient(n, NavigateWaypoints, ACTION_NAME)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, n)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---- helpers ---------------------------------------------------------

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.obstacles is not None and time.time() - self.last_obstacle_pub > 0.2:
            header = Header()
            header.frame_id = 'map'
            header.stamp = self.node.get_clock().now().to_msg()
            self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, self.obstacles))
            self.__class__.last_obstacle_pub = time.time()

    def spin_until(self, predicate, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_once()
            if predicate():
                return True
        return False

    def wait_ready(self):
        ok = self.spin_until(
            lambda: self.last is not None and
            self.last.state == NavigationStatus.STATE_READY and
            self.gt is not None and self.action.server_is_ready() and
            self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()), STARTUP_TIMEOUT)
        self.assertTrue(ok, 'the planner did not get ready')

    def run_sequence(self, points, allow_skip=False):
        """Sends a sequence and waits for its result. Returns (result, feedback, first
        status index, first track index, first follower status index)."""
        start = (len(self.messages), len(self.track), len(self.follower))
        feedback = []
        fut = self.action.send_goal_async(
            make_waypoints(points, allow_skip),
            feedback_callback=lambda f: feedback.append(f.feedback.state))
        self.assertTrue(self.spin_until(fut.done, 10.0), 'goal not answered')
        handle = fut.result()
        self.assertTrue(handle.accepted, 'waypoints rejected')
        res_fut = handle.get_result_async()
        self.spin_until(res_fut.done, SEQUENCE_TIMEOUT)
        self.assertTrue(res_fut.done(), 'the sequence did not end in time')
        return res_fut.result(), feedback, start

    def min_dist(self, track, p):
        return min(math.hypot(x - p[0], y - p[1]) for _, x, y in track)

    # ---- tests -----------------------------------------------------------

    def test_a_sequence_driven_through(self):
        self.wait_ready()
        res, feedback, (i_status, i_track, i_follower) = self.run_sequence(ROUTE)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(res.result.state.navigation_status, NavigationFinalStatus.STATUS_SUCCESS)
        self.assertEqual(res.result.state.reason, 'reached')

        # One goal, with the waypoint counters going up to the last one:
        msgs = [m for m in self.messages[i_status:] if m.goal_id != 0]
        self.assertEqual(len({m.goal_id for m in msgs}), 1)
        self.assertTrue(all(m.total_waypoints == len(ROUTE) for m in msgs))
        reached = [m.reached_waypoints for m in msgs]
        self.assertEqual(reached, sorted(reached), 'waypoint counter went back')
        self.assertEqual(set(reached), set(range(len(ROUTE))))
        # The status goal is the current waypoint:
        for m in msgs:
            wp = ROUTE[min(m.reached_waypoints, len(ROUTE) - 1)]
            self.assertAlmostEqual(m.goal.pose.position.x, wp[0], places=2)
            self.assertAlmostEqual(m.goal.pose.position.y, wp[1], places=2)
        self.assertTrue(feedback)
        self.assertTrue(all(f.total_waypoints == len(ROUTE) for f in feedback))
        self.assertEqual(max(f.reached_waypoints for f in feedback), len(ROUTE) - 1)

        # Through every waypoint, ending at the last one:
        track = self.track[i_track:]
        for wp in ROUTE:
            d = self.min_dist(track, wp)
            print(f'Closest approach to waypoint {wp}: {d:.2f} m', flush=True)
            self.assertLess(d, WAYPOINT_GT_TOLERANCE)
        self.assertLess(math.hypot(self.gt[0] - ROUTE[-1][0], self.gt[1] - ROUTE[-1][1]),
                        WAYPOINT_GT_TOLERANCE)

        # Without stopping at the first intermediate waypoint:
        speeds = []
        for (t0, x0, y0), (t1, x1, y1) in zip(track, track[1:]):
            if t1 - t0 > 1e-3 and math.hypot(x1 - ROUTE[0][0], y1 - ROUTE[0][1]) < NEAR_WAYPOINT:
                speeds.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0))
        self.assertTrue(speeds, 'the robot never got close to the first waypoint')
        # (smoothed over a few ground truth samples)
        k = 5
        smooth = [sum(speeds[i:i + k]) / k for i in range(max(1, len(speeds) - k + 1))]
        print(f'Min speed near the first waypoint: {min(smooth):.2f} m/s', flush=True)
        self.assertGreater(min(smooth), MIN_SPEED_AT_WAYPOINT)

        # The follower only reached a goal at the end (its status may come after
        # the action result):
        def reached_goal():
            return [s for _, s in self.follower[i_follower:] if s.status == 'ReachedGoal']

        self.assertTrue(self.spin_until(reached_goal, 5.0), 'no ReachedGoal from the follower')
        reached_goal = reached_goal()
        self.assertTrue(all(s.goal_id == msgs[0].goal_id for s in reached_goal))
        self.assertEqual(self.collisions, 0)

    def test_b_skips_a_waypoint_in_collision(self):
        self.wait_ready()
        # A solid box on the first waypoint, known to the planner:
        wp = ROUTE_BACK[0]
        self.__class__.obstacles = [(wp[0] + dx * 0.1, wp[1] + dy * 0.1, 0.2)
                                    for dx in range(-5, 6) for dy in range(-5, 6)]
        self.spin_until(lambda: False, 2.0)
        res, _, (i_status, _, _) = self.run_sequence(ROUTE_BACK, allow_skip=True)
        self.__class__.obstacles = None
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.state.message)
        msgs = [m for m in self.messages[i_status:] if m.goal_id != 0]
        # Never driving to the blocked waypoint:
        self.assertTrue(all(m.reached_waypoints == 1 for m in msgs),
                        {m.reached_waypoints for m in msgs})
        self.assertLess(math.hypot(self.gt[0] - ROUTE_BACK[-1][0],
                                   self.gt[1] - ROUTE_BACK[-1][1]), WAYPOINT_GT_TOLERANCE)
        self.assertEqual(self.collisions, 0)

    def test_c_empty_obstacle_source(self):
        self.wait_ready()
        self.__class__.obstacles = []
        self.spin_until(lambda: False, 2.0)
        res, _, _ = self.run_sequence([HOME])
        self.__class__.obstacles = None
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.state.message)
        self.assertLess(math.hypot(self.gt[0] - HOME[0], self.gt[1] - HOME[1]),
                        WAYPOINT_GT_TOLERANCE)
        self.assertEqual(self.collisions, 0)
