# Integration tests of the planner goal manager against misbehaving or late
# inputs, end-to-end in simulation (mvsim, headless, diffdrive robot):
#  - A follower status that refers to an old path (a stale ReachedGoal for a
#    previous revision, or for another goal) must not end the current goal.
#  - A box that appears on the best path as soon as it is known, seen by the
#    planner only (so it may hit the paths just found, before being published
#    or right after), is avoided, and the goal is reached without collisions.
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
import rclpy.time
from action_msgs.msg import GoalStatus
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mrpt_nav_interfaces.action import NavigateGoal
from mrpt_nav_interfaces.msg import FollowerStatus, NavigationFinalStatus, ReferencePath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String

STATUS_TOPIC = '/mrpt_trajectory_follower/status'
STATUS_EX_TOPIC = '/mrpt_trajectory_follower/status_ex'
REF_PATH_TOPIC = '/mrpt_trajectory_follower/reference_path'
ROBOT_SHAPE_TOPIC = '/mrpt_tps_astar_planner_node/robot_shape'
ACTION_NAME = '/mrpt_tps_astar_planner_node/navigate_goal'
VIRTUAL_OBSTACLES_TOPIC = '/test_virtual_obstacles'
REFERENCE_PATH_TOPIC = '/waypoints_reference'

# Short, and with exact (slower) A*, so partial paths are published before the
# search reaches the goal:
ANYTIME_FIRST_PATH_TIME = 0.01  # [s]
PLANNER_PARAMETERS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'planner-test-exact-astar.yaml')

STARTUP_TIMEOUT = 300.0     # [s] nodes ready (PTG collision grids may be built)
GOAL_TIMEOUT = 150.0        # [s] per goal
MAX_TIME_TO_MOVE = 6.0      # [s] from a distant goal until the robot moves

# Goals (x [m], y [m], yaw [deg]), map frame. The robot starts at (30, 30).
FAR_GOAL = (9.20, 17.95, -165.0)
OTHER_GOAL = (39.68, 40.22, -144.0)
HOME_GOAL = (30.0, 30.0, 180.0)

# Virtual obstacle: a square of this half-size [m], this far ahead [m] of the
# robot along its path:
OBSTACLE_HALF_SIZE = 0.4
OBSTACLE_AHEAD = 6.0
# Minimum clearance [m] of the robot center to the obstacle square (about the
# robot half-width):
MIN_CLEARANCE = 0.25


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
                'anytime_first_path_time': str(ANYTIME_FIRST_PATH_TIME),
                'planner_parameters': PLANNER_PARAMETERS,
                'pf_params_overrides_file': os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 'pf-test-overrides.yaml'),
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose(g):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = g[0]
    msg.pose.position.y = g[1]
    yaw = math.radians(g[2])
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


class TestGoalManagerRobustness(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('goal_manager_robustness_test')
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.status = None
        cls.statuses = []
        cls.collisions = 0
        cls.ref_path = None
        cls.robot_shape = None
        cls.references = []

        def on_gt(m):
            p = m.pose.pose
            speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
            cls.gt = (p.position.x, p.position.y, yaw_of(p.orientation), speed)

        def on_status(m):
            if m.data != cls.status:
                cls.statuses.append(m.data)
            cls.status = m.data

        def on_collision(m):
            if m.data:
                cls.collisions += 1

        def on_ref_path(m):
            cls.ref_path = [(q.pose.position.x, q.pose.position.y) for q in m.poses]

        def on_shape(m):
            cls.robot_shape = [(p.x, p.y) for p in m.polygon.points]

        def on_reference(m):
            cls.references.append((m.goal_id, m.revision, m.is_partial, len(m.path.poses)))

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        n.create_subscription(Path, REF_PATH_TOPIC, on_ref_path, latched)
        n.create_subscription(PolygonStamped, ROBOT_SHAPE_TOPIC, on_shape, latched)
        n.create_subscription(ReferencePath, REFERENCE_PATH_TOPIC, on_reference, 10)
        cls.pub_goal = n.create_publisher(PoseStamped, '/goal_pose', 10)
        cls.pub_status_ex = n.create_publisher(FollowerStatus, STATUS_EX_TOPIC, latched)
        cls.pub_obstacles = n.create_publisher(PointCloud2, VIRTUAL_OBSTACLES_TOPIC, 10)
        cls.action = ActionClient(n, NavigateGoal, ACTION_NAME)
        cls.obstacle = None  # (x, y) center, once placed
        cls.last_obstacle_pub = 0.0

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, duration):
        t0 = time.time()
        while time.time() - t0 < duration:
            self.spin_once()

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        # Keep publishing the virtual obstacle, if any:
        if self.obstacle and time.time() - self.last_obstacle_pub > 0.2:
            self.publish_obstacle()

    def spin_until(self, predicate, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_once()
            if predicate():
                return True
        return False

    def publish_obstacle(self):
        cx, cy = self.obstacle
        h = OBSTACLE_HALF_SIZE
        pts = []
        steps = 8
        for i in range(steps + 1):
            t = -h + 2 * h * i / steps
            pts += [(cx + t, cy - h, 0.2), (cx + t, cy + h, 0.2),
                    (cx - h, cy + t, 0.2), (cx + h, cy + t, 0.2)]
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.node.get_clock().now().to_msg()
        self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, pts))
        self.last_obstacle_pub = time.time()

    def clearance_to_obstacle(self):
        """Distance from the robot center to the obstacle square."""
        cx, cy = self.obstacle
        dx = max(abs(self.gt[0] - cx) - OBSTACLE_HALF_SIZE, 0.0)
        dy = max(abs(self.gt[1] - cy) - OBSTACLE_HALF_SIZE, 0.0)
        return math.hypot(dx, dy)

    def send_action_goal(self, g):
        goal = NavigateGoal.Goal()
        goal.target = make_pose(g)
        goal.target.header.stamp = self.node.get_clock().now().to_msg()
        fut = self.action.send_goal_async(goal)
        self.assertTrue(self.spin_until(fut.done, 10.0), 'goal not answered')
        handle = fut.result()
        self.assertTrue(handle.accepted, 'goal rejected')
        return handle

    def wait_result(self, handle, timeout):
        fut = handle.get_result_async()
        self.spin_until(fut.done, timeout)
        return fut.result() if fut.done() else None

    def point_ahead(self, dist):
        """Point of the reference path `dist` meters ahead of the robot."""
        path = self.ref_path
        i0 = min(range(len(path)),
                 key=lambda i: math.hypot(path[i][0] - self.gt[0], path[i][1] - self.gt[1]))
        s = 0.0
        for i in range(i0 + 1, len(path)):
            s += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if s >= dist:
                return path[i]
        return None

    def wait_ready(self):
        ok = self.spin_until(
            lambda: self.robot_shape is not None and self.gt is not None and
            self.status is not None and self.action.server_is_ready(), STARTUP_TIMEOUT)
        self.assertTrue(ok, 'navigation nodes not ready')
        self.spin(10.0)  # let the localization converge

    def fake_status(self, status, goal_id, revision):
        msg = FollowerStatus()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status = status
        msg.has_goal_id = True
        msg.goal_id = goal_id
        msg.revision = revision
        self.pub_status_ex.publish(msg)

    def test_a_stale_follower_status_is_ignored(self):
        self.wait_ready()
        self.references.clear()
        self.__class__.collisions = 0
        handle = self.send_action_goal(FAR_GOAL)
        self.assertTrue(self.spin_until(lambda: self.gt[3] > 0.1, 20.0), 'not moving')
        self.assertTrue(self.spin_until(lambda: len(self.references) > 0, 10.0))
        goal_id, revision = self.references[-1][0], self.references[-1][1]
        self.assertGreater(revision, 0)

        # The status of an old revision of this goal, and of another goal,
        # both claiming the goal was reached, arrive while driving:
        for _ in range(5):
            self.fake_status('ReachedGoal', goal_id, 0)
            self.fake_status('ReachedGoal', goal_id + 100, revision)
            self.fake_status('Failed', goal_id, revision - 1)
            self.spin(0.5)
        result_fut = handle.get_result_async()
        self.assertFalse(result_fut.done(), 'a stale status ended the goal')
        self.assertGreater(self.gt[3], 0.05, 'the robot stopped')

        # ...and the goal ends only when it really is reached:
        self.spin_until(result_fut.done, GOAL_TIMEOUT)
        self.assertTrue(result_fut.done(), 'navigation did not end in time')
        res = result_fut.result()
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(res.result.state.navigation_status,
                         NavigationFinalStatus.STATUS_SUCCESS)
        self.assertLess(math.hypot(self.gt[0] - FAR_GOAL[0], self.gt[1] - FAR_GOAL[1]), 0.5,
                        'the goal was reported reached far from it')
        self.assertEqual(self.collisions, 0)

    def test_b_box_dropped_on_the_first_path(self):
        self.wait_ready()
        # Back to a known place first (the previous test ended at FAR_GOAL):
        self.send_action_goal(HOME_GOAL)
        self.assertTrue(self.spin_until(lambda: self.status == 'ReachedGoal' and math.hypot(
            self.gt[0] - HOME_GOAL[0], self.gt[1] - HOME_GOAL[1]) < 0.5, GOAL_TIMEOUT),
            'robot did not return home')
        self.spin(2.0)
        self.__class__.collisions = 0
        self.__class__.ref_path = None
        self.references.clear()

        handle = self.send_action_goal(FAR_GOAL)
        # As soon as a path is known, drop a box close ahead on it:
        self.assertTrue(self.spin_until(lambda: self.ref_path is not None, 20.0),
                        'no path published')
        p = self.point_ahead(4.0)
        self.assertIsNotNone(p, 'path too short to place the box')
        self.__class__.obstacle = p
        self.publish_obstacle()

        min_clearance = float('inf')
        result_fut = handle.get_result_async()
        t0 = time.time()
        while not result_fut.done() and time.time() - t0 < GOAL_TIMEOUT:
            self.spin_once()
            min_clearance = min(min_clearance, self.clearance_to_obstacle())
        self.__class__.obstacle = None

        self.assertTrue(result_fut.done(), 'navigation did not end in time')
        res = result_fut.result()
        print(f'Box on the first path: {res.status}, min clearance {min_clearance:.2f} m, '
              f'statuses: {self.statuses}', flush=True)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreater(min_clearance, MIN_CLEARANCE, 'the robot drove into the box')
        self.assertEqual(self.collisions, 0)
        self.assertLess(math.hypot(self.gt[0] - FAR_GOAL[0], self.gt[1] - FAR_GOAL[1]), 0.5)
