# Integration tests of the navigation status and feedback of the planner,
# end-to-end in simulation (mvsim, headless, diffdrive robot):
#  - The status goes from INITIALIZING (with progress) to READY at startup, as
#    seen by a subscriber with the latched QoS. The PTG cache is empty, so the
#    PTG collision grids are built. A topic goal sent while INITIALIZING is
#    queued, and reached once ready.
#  - A normal goal goes through PLANNING and FOLLOWING states, with a remaining
#    path length that decreases, and ends SUCCEEDED, then the status goes back
#    to READY. The action feedback and result carry the same information.
#  - A goal in collision ends FAILED with its reason, as a topic and action goal.
#  - A goal that cannot be reached (walled in) turns STALLED before it is given
#    up (FAILED, no_progress).
#  - A canceled goal ends CANCELED.
#
# Run it with:
#   colcon test --packages-select mrpt_navigation

import math
import os
import shutil
import tempfile
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
from geometry_msgs.msg import PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mrpt_nav_interfaces.action import NavigateGoal
from mrpt_nav_interfaces.msg import NavigationFinalStatus, NavigationStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import MarkerArray

PLANNER_NS = '/mrpt_tps_astar_planner_node'
STATUS_TOPIC = PLANNER_NS + '/navigation_status'
MARKERS_TOPIC = PLANNER_NS + '/navigation_markers'
ACTION_NAME = PLANNER_NS + '/navigate_goal'
VIRTUAL_OBSTACLES_TOPIC = '/test_virtual_obstacles'

# Short, so the stall of a walled-in goal is quick: a warning is given at half
# of it (the default no_progress_warn_fraction).
NO_PROGRESS_TIMEOUT = 16.0  # [s]

STARTUP_TIMEOUT = 300.0     # [s] nodes ready (PTG collision grids are built)
GOAL_TIMEOUT = 150.0        # [s] per goal

# Goals (x [m], y [m], yaw [deg]), map frame. The robot starts at (30, 30).
FIRST_GOAL = (39.68, 40.22, -144.0)
HOME_GOAL = (30.0, 30.0, 180.0)
WALLED_GOAL = (23.47, 32.35, -65.0)
FAR_GOAL = (9.20, 17.95, -165.0)

# Ground truth distance [m] to a goal accepted as reached (the follower stops
# using the localized pose, whose error is not what these tests check):
GOAL_GT_TOLERANCE = 1.5

# Half-size [m] of the walls around a goal that cannot be reached:
WALL_HALF_SIZE = 1.6


CACHE_DIRS = []


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    demo = os.path.join(
        get_package_share_directory('mrpt_tutorials'), 'launch',
        'demo_astar_trajectory_follower_gridmap.launch.py')
    # An empty PTG cache, so the collision grids are built (INITIALIZING):
    cache_dir = tempfile.mkdtemp(prefix='mrpt_nav_test_cache_')
    CACHE_DIRS.append(cache_dir)
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('XDG_CACHE_HOME', cache_dir),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo),
            launch_arguments={
                'robot': 'diffdrive',
                'speed_limit': '1.0',
                'use_rviz': 'False',
                'planner_extra_obstacles_topic': VIRTUAL_OBSTACLES_TOPIC,
                'no_progress_timeout': str(NO_PROGRESS_TIMEOUT),
                'pf_params_overrides_file': os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 'pf-test-overrides.yaml'),
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


@pytest.fixture(scope='module', autouse=True)
def remove_cache_dir():
    yield
    for d in CACHE_DIRS:
        shutil.rmtree(d, ignore_errors=True)


def make_pose(g):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = g[0]
    msg.pose.position.y = g[1]
    yaw = math.radians(g[2])
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def S(name):
    return getattr(NavigationStatus, 'STATE_' + name)


class TestNavigationStatus(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigation_status_test')
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        latched_all = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.collisions = 0
        cls.last = None             # latest NavigationStatus
        cls.messages = []           # all of them
        cls.markers = None          # latest MarkerArray
        cls.obstacles = None        # points of the virtual obstacles, if any
        cls.last_obstacle_pub = 0.0

        def on_gt(m):
            p = m.pose.pose
            cls.gt = (p.position.x, p.position.y)

        def on_status(m):
            cls.last = m
            cls.messages.append(m)

        def on_markers(m):
            cls.markers = m

        def on_collision(m):
            if m.data:
                cls.collisions += 1

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(NavigationStatus, STATUS_TOPIC, on_status, latched_all)
        n.create_subscription(MarkerArray, MARKERS_TOPIC, on_markers, latched)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        cls.pub_goal = n.create_publisher(PoseStamped, '/goal_pose', 10)
        cls.pub_obstacles = n.create_publisher(PointCloud2, VIRTUAL_OBSTACLES_TOPIC, 10)
        cls.action = ActionClient(n, NavigateGoal, ACTION_NAME)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---- helpers ---------------------------------------------------------

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        # Keep publishing the virtual obstacles, if any:
        if self.obstacles and time.time() - self.last_obstacle_pub > 0.2:
            header = Header()
            header.frame_id = 'map'
            header.stamp = self.node.get_clock().now().to_msg()
            self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, self.obstacles))
            self.__class__.last_obstacle_pub = time.time()

    def spin(self, duration):
        t0 = time.time()
        while time.time() - t0 < duration:
            self.spin_once()

    def spin_until(self, predicate, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_once()
            if predicate():
                return True
        return False

    def set_obstacles(self, pts):
        self.__class__.obstacles = pts

    def states_since(self, index):
        """Sequence of (state, reason) of the status messages from `index`, without repeats."""
        out = []
        for m in self.messages[index:]:
            item = (m.state, m.reason)
            if not out or out[-1] != item:
                out.append(item)
        return out

    def wait_ready_for_goals(self):
        ok = self.spin_until(
            lambda: self.last is not None and self.last.state == S('READY') and
            self.gt is not None and self.action.server_is_ready(), STARTUP_TIMEOUT)
        self.assertTrue(ok, 'the planner did not get ready')

    def wait_state(self, state, timeout):
        return self.spin_until(lambda: self.last.state == state, timeout)

    def send_action_goal(self, g, feedback_cb=None):
        goal = NavigateGoal.Goal()
        goal.target = make_pose(g)
        goal.target.header.stamp = self.node.get_clock().now().to_msg()
        fut = self.action.send_goal_async(goal, feedback_callback=feedback_cb)
        self.assertTrue(self.spin_until(fut.done, 10.0), 'goal not answered')
        handle = fut.result()
        self.assertTrue(handle.accepted, 'goal rejected')
        return handle

    def wait_result(self, handle, timeout):
        fut = handle.get_result_async()
        self.spin_until(fut.done, timeout)
        self.assertTrue(fut.done(), 'navigation did not end in time')
        return fut.result()

    def dist_to(self, g):
        return math.hypot(self.gt[0] - g[0], self.gt[1] - g[1])

    # ---- tests -----------------------------------------------------------

    def test_a_initializing_then_ready_with_a_queued_goal(self):
        # (the status is latched: we get the current one even if we are late)
        self.assertTrue(self.spin_until(lambda: self.last is not None, 60.0),
                        'no navigation status published')
        first = self.messages[0]
        self.assertEqual(first.state, S('INITIALIZING'))
        self.assertEqual(first.state_name, 'INITIALIZING')
        self.assertEqual(first.level, NavigationStatus.LEVEL_WARN)
        self.assertEqual(first.goal_id, 0)

        # A goal sent while the PTGs are being built is queued:
        self.pub_goal.publish(make_pose(FIRST_GOAL))
        self.assertTrue(self.spin_until(
            lambda: any(m.reason == 'goal_queued' for m in self.messages) or
            self.last.state != S('INITIALIZING'), 30.0))
        self.assertTrue(
            any(m.reason == 'goal_queued' for m in self.messages),
            'the goal sent while initializing was not queued, states: ' +
            str(self.states_since(0)))
        queued = [m for m in self.messages if m.reason == 'goal_queued'][-1]
        self.assertEqual(queued.state, S('INITIALIZING'))
        self.assertEqual(queued.goal_id, 1)
        self.assertAlmostEqual(queued.goal.pose.position.x, FIRST_GOAL[0], places=3)
        self.assertFalse(queued.goal_from_action)

        # ...then the planner gets ready, and the queued goal is reached:
        self.assertTrue(self.spin_until(
            lambda: any(m.state == S('SUCCEEDED') or m.state == S('FAILED') for m in
                        self.messages), STARTUP_TIMEOUT + GOAL_TIMEOUT),
            'the queued goal did not end, states: ' + str(self.states_since(0)))
        states = self.states_since(0)
        print(f'States for the queued goal: {states}', flush=True)
        names = [s for s, _ in states]
        self.assertEqual(names[-1], S('SUCCEEDED'), f'states: {states}')
        self.assertEqual(states[-1][1], 'reached')
        for expected in (S('INITIALIZING'), S('PLANNING')):
            self.assertIn(expected, names)
        self.assertTrue(S('FOLLOWING') in names or S('FOLLOWING_PARTIAL') in names)
        self.assertLess(names.index(S('INITIALIZING')), names.index(S('PLANNING')))
        self.assertNotIn(S('INIT_FAILED'), names)
        self.assertNotIn(S('FAILED'), names)

        # Initialization progress is reported (with the macro of newer mpp):
        init = [m for m in self.messages if m.state == S('INITIALIZING')]
        self.assertTrue(all(0.0 <= m.init_progress <= 1.0 for m in init))
        self.assertTrue(max(m.init_progress for m in init) > 0.0 or
                        any(m.init_detail for m in init),
                        'no initialization progress was reported')

        # The remaining path length decreases while following:
        remaining = [m.remaining_path_length for m in self.messages
                     if m.goal_id == 1 and m.state == S('FOLLOWING') and
                     not math.isnan(m.remaining_path_length)]
        self.assertGreater(len(remaining), 2)
        self.assertGreater(remaining[0], remaining[-1] + 3.0)
        etas = [m.eta for m in self.messages
                if m.goal_id == 1 and m.state == S('FOLLOWING') and not math.isnan(m.eta)]
        self.assertTrue(all(e >= 0 for e in etas))

        self.assertLess(self.dist_to(FIRST_GOAL), GOAL_GT_TOLERANCE)
        self.assertEqual(self.collisions, 0)

        # The goal is shown while it is handled, and the status goes back to READY:
        self.assertTrue(self.wait_state(S('READY'), 30.0), 'status did not go back to READY')
        self.assertEqual(self.last.goal_id, 0)

    def test_b_normal_goal_with_action_feedback(self):
        self.wait_ready_for_goals()
        start = len(self.messages)
        feedback = []
        handle = self.send_action_goal(HOME_GOAL, lambda f: feedback.append(f.feedback.state))

        # The HUD markers show the goal while navigating:
        self.assertTrue(self.spin_until(
            lambda: self.markers is not None and any(
                m.ns == 'goal' and m.action == 0 for m in self.markers.markers), 30.0),
            'no goal marker while navigating')
        self.assertTrue(any(m.ns == 'status_text' and m.text.startswith('#')
                            for m in self.markers.markers))

        res = self.wait_result(handle, GOAL_TIMEOUT)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(res.result.state.navigation_status,
                         NavigationFinalStatus.STATUS_SUCCESS)
        self.assertEqual(res.result.state.reason, 'reached')
        self.assertTrue(res.result.state.message)

        # The final status message may arrive after the action result:
        self.spin_until(
            lambda: self.states_since(start) and
            self.states_since(start)[-1][0] == S('SUCCEEDED'), 5.0)
        states = self.states_since(start)
        print(f'States for a normal goal: {states}', flush=True)
        names = [s for s, _ in states]
        self.assertEqual(names[0], S('PLANNING'))
        self.assertEqual(names[-1], S('SUCCEEDED'))
        self.assertTrue(S('FOLLOWING') in names or S('FOLLOWING_PARTIAL') in names)
        self.assertNotIn(S('FAILED'), names)
        goal_ids = {m.goal_id for m in self.messages[start:] if m.goal_id != 0}
        self.assertEqual(len(goal_ids), 1)
        self.assertTrue(all(m.goal_from_action for m in self.messages[start:] if m.goal_id != 0))

        # Feedback comes from the same status:
        self.assertGreater(len(feedback), 2)
        valid_names = {'PLANNING', 'FOLLOWING', 'FOLLOWING_PARTIAL', 'REPLANNING', 'BLOCKED',
                       'STALLED'}
        self.assertTrue(all(f.status in valid_names for f in feedback),
                        {f.status for f in feedback})
        followed = [f.remaining_path_length for f in feedback
                    if f.status == 'FOLLOWING' and not math.isnan(f.remaining_path_length)]
        self.assertGreater(len(followed), 2)
        self.assertGreater(followed[0], followed[-1] + 3.0)
        self.assertTrue(all(f.time_since_progress >= 0 for f in feedback))

        self.assertLess(self.dist_to(HOME_GOAL), GOAL_GT_TOLERANCE)
        self.assertTrue(self.wait_state(S('READY'), 30.0))
        self.assertEqual(self.collisions, 0)

    def test_c_goal_in_collision_fails_with_its_reason(self):
        self.wait_ready_for_goals()
        # A solid box around the goal, known to the planner:
        goal = FIRST_GOAL
        box = [(goal[0] + dx * 0.1, goal[1] + dy * 0.1, 0.2)
               for dx in range(-5, 6) for dy in range(-5, 6)]
        self.set_obstacles(box)
        self.spin(2.0)

        # As an action goal:
        handle = self.send_action_goal(goal)
        res = self.wait_result(handle, 30.0)
        self.assertEqual(res.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(res.result.state.navigation_status, NavigationFinalStatus.STATUS_ERROR)
        self.assertEqual(res.result.state.reason, 'goal_in_collision')
        self.assertTrue(res.result.state.message)
        self.assertTrue(self.spin_until(lambda: self.last.state == S('FAILED'), 10.0))
        self.assertEqual(self.last.reason, 'goal_in_collision')
        self.assertEqual(self.last.level, NavigationStatus.LEVEL_ERROR)
        self.assertEqual(self.last.state_name, 'FAILED')
        self.assertTrue(self.wait_state(S('READY'), 30.0))

        # As a topic goal:
        start = len(self.messages)
        self.pub_goal.publish(make_pose(goal))
        self.assertTrue(self.spin_until(
            lambda: any(m.state == S('FAILED') for m in self.messages[start:]), 30.0))
        failed = [m for m in self.messages[start:] if m.state == S('FAILED')][-1]
        self.assertEqual(failed.reason, 'goal_in_collision')
        self.assertFalse(failed.goal_from_action)
        self.set_obstacles(None)
        self.assertTrue(self.wait_state(S('READY'), 30.0))

    def test_d_unreachable_goal_stalls_before_failing(self):
        self.wait_ready_for_goals()
        # Walls around the goal, known to the planner:
        goal = WALLED_GOAL
        h = WALL_HALF_SIZE
        steps = int(2 * h / 0.05)
        wall = []
        for i in range(steps + 1):
            t = -h + i * 0.05
            wall += [(goal[0] + t, goal[1] - h, 0.2), (goal[0] + t, goal[1] + h, 0.2),
                     (goal[0] - h, goal[1] + t, 0.2), (goal[0] + h, goal[1] + t, 0.2)]
        self.set_obstacles(wall)
        self.spin(2.0)

        start = len(self.messages)
        handle = self.send_action_goal(goal)
        res = self.wait_result(handle, GOAL_TIMEOUT)
        self.set_obstacles(None)
        states = self.states_since(start)
        print(f'States for a walled-in goal: {states}', flush=True)
        self.assertEqual(res.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(res.result.state.navigation_status, NavigationFinalStatus.STATUS_ERROR)
        self.assertEqual(res.result.state.reason, 'no_progress')

        names = [s for s, _ in states]
        self.assertIn(S('STALLED'), names, f'no warning before giving up: {states}')
        self.assertEqual(names[-1], S('FAILED'))
        self.assertLess(names.index(S('STALLED')), names.index(S('FAILED')))
        stalled = [m for m in self.messages[start:] if m.state == S('STALLED')]
        self.assertEqual(stalled[0].level, NavigationStatus.LEVEL_WARN)
        self.assertIn(stalled[0].reason, ('searching', 'no_progress', 'waiting_partial_path'))
        self.assertGreater(stalled[0].time_since_progress, 0.4 * NO_PROGRESS_TIMEOUT)
        self.assertLess(stalled[0].time_since_progress, NO_PROGRESS_TIMEOUT)
        self.assertAlmostEqual(stalled[0].no_progress_timeout, NO_PROGRESS_TIMEOUT, places=3)
        self.assertTrue(self.wait_state(S('READY'), 30.0))
        self.assertEqual(self.collisions, 0)

    def test_e_canceled_goal(self):
        self.wait_ready_for_goals()
        handle = self.send_action_goal(FAR_GOAL)
        self.assertTrue(self.spin_until(
            lambda: self.last.state in (S('FOLLOWING'), S('FOLLOWING_PARTIAL')), 60.0),
            'not following a path')
        fut = handle.cancel_goal_async()
        self.assertTrue(self.spin_until(fut.done, 10.0))
        res = self.wait_result(handle, 30.0)
        self.assertEqual(res.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(res.result.state.navigation_status, NavigationFinalStatus.STATUS_CANCELLED)
        self.assertEqual(res.result.state.reason, 'canceled')
        self.assertTrue(self.spin_until(lambda: self.last.state == S('CANCELED'), 10.0))
        self.assertEqual(self.last.reason, 'canceled')
        self.assertTrue(self.wait_state(S('READY'), 30.0))
        self.assertEqual(self.collisions, 0)
