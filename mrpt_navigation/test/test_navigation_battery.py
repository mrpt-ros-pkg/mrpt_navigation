# Integration test: end-to-end navigation in simulation (mvsim), headless.
#
# Launches demo_astar_trajectory_follower_gridmap.launch.py (map server, A*
# planner, PF localization, point cloud pipeline, trajectory follower, mvsim)
# and drives the robot through a sequence of A->B goals, checking that the
# simulator reports no collisions, that the robot never moves far from its
# reference path, that each navigation ends (reached or reported as failed) in
# time, and that goals reported as reached really are. How many goals must be
# reached depends on the robot (see CRITERIA).
#
# Environment variables:
#  - MRPT_NAV_TEST_ROBOT: 'diffdrive' (default) or 'ackermann'.
#  - MRPT_NAV_TEST_SPEED_LIMIT: follower speed limit [m/s] (default: 1.0).
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
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Polygon, PolygonStamped, PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

ROBOT = os.environ.get('MRPT_NAV_TEST_ROBOT', 'diffdrive')
SPEED_LIMIT = os.environ.get('MRPT_NAV_TEST_SPEED_LIMIT', '1.0')

# Goals (x [m], y [m], yaw [deg]) in the map frame, starting from the robot
# initial pose in the mvsim world (30, 30, 0 deg). Long and short trips,
# reverse maneuvers to reach goal headings, and the far ends of the map.
GOALS = [
    (23.47, 32.35, -65.0),
    (39.68, 40.22, -144.0),
    (9.20, 17.95, -165.0),
    (8.81, 0.31, -103.0),
    (30.0, 30.0, 180.0),
]

# Acceptance criteria:
STARTUP_TIMEOUT = 300.0     # [s] nodes ready (PTG collision grids may be built)
GOAL_TIMEOUT = 150.0        # [s] per goal
MAX_PATH_DEVIATION = 1.5    # [m] ground truth vs reference path, while moving
MAX_STARTUP_LOC_ERROR = 0.4       # [m] localization converged before the first goal
MAX_STARTUP_LOC_YAW_ERROR = 10.0  # [deg]

# Per robot: minimum number of goals reached, and max final errors of reached
# goals (ground truth vs goal, so they include the localization error). The
# car does not reliably complete short maneuvers near goals yet (it then stops
# and reports a failure), so it must reach only some of them.
CRITERIA = {
    'diffdrive': {'min_reached': len(GOALS), 'max_pos_err': 0.6, 'max_yaw_err': 30.0},
    'ackermann': {'min_reached': 2, 'max_pos_err': 1.0, 'max_yaw_err': 30.0},
}

STATUS_TOPIC = '/mrpt_trajectory_follower/status'
REF_PATH_TOPIC = '/mrpt_trajectory_follower/reference_path'
ROBOT_SHAPE_TOPIC = '/mrpt_tps_astar_planner_node/robot_shape'


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
                'robot': ROBOT,
                'speed_limit': SPEED_LIMIT,
                'use_rviz': 'False',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def dist_to_polyline(p, pts):
    best = float('inf')
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        sx, sy = bx - ax, by - ay
        l2 = sx * sx + sy * sy
        t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * sx + (p[1] - ay) * sy) / l2))
        best = min(best, math.hypot(p[0] - ax - t * sx, p[1] - ay - t * sy))
    return best


def point_in_polygon(p, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > p[1]) != (yj > p[1]) and p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


class NavigationBattery(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigation_battery_test')
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.status = None
        cls.status_stamp = 0.0
        cls.collisions = 0
        cls.ref_path = None
        cls.ref_path_stamp = 0.0
        cls.robot_shape = None
        cls.chassis = None

        def on_gt(m):
            p = m.pose.pose
            speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
            cls.gt = (p.position.x, p.position.y, yaw_of(p.orientation), speed,
                      time.time())

        def on_status(m):
            cls.status = m.data
            cls.status_stamp = time.time()

        def on_collision(m):
            if m.data:
                cls.collisions += 1

        def on_ref_path(m):
            cls.ref_path = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
            cls.ref_path_stamp = time.time()

        def on_shape(m):
            cls.robot_shape = [(p.x, p.y) for p in m.polygon.points]

        def on_chassis(m):
            cls.chassis = [(p.x, p.y) for p in m.points]

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        n.create_subscription(Path, REF_PATH_TOPIC, on_ref_path, latched)
        n.create_subscription(PolygonStamped, ROBOT_SHAPE_TOPIC, on_shape, latched)
        n.create_subscription(Polygon, '/chassis_polygon', on_chassis, latched)
        cls.pub_goal = n.create_publisher(PoseStamped, '/goal_pose', 10)

        from tf2_ros import Buffer, TransformListener
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, n)

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

    def localized(self):
        """Whether the localization estimate agrees with the ground truth."""
        if self.gt is None:
            return False
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return False
        q = t.transform.rotation
        return (math.hypot(t.transform.translation.x - self.gt[0],
                           t.transform.translation.y - self.gt[1]) < MAX_STARTUP_LOC_ERROR and
                abs(math.degrees(angle_diff(yaw_of(q), self.gt[2]))) < MAX_STARTUP_LOC_YAW_ERROR)

    def send_goal(self, g):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = g[0]
        msg.pose.position.y = g[1]
        yaw = math.radians(g[2])
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.pub_goal.publish(msg)

    def navigate_to(self, g):
        """Sends a goal and waits for the outcome. Returns a dict of metrics."""
        t_sent = time.time()
        self.send_goal(g)
        res = {'max_dev': 0.0, 'statuses': []}
        t_reached = None
        while time.time() - t_sent < GOAL_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            # Only statuses after the new path started being followed (or a
            # planning failure) refer to this goal:
            started = self.ref_path_stamp > t_sent or (
                self.status in ('Canceled', 'Failed') and self.status_stamp > t_sent)
            fresh = started and self.status_stamp > max(t_sent, self.ref_path_stamp)
            if fresh and (not res['statuses'] or res['statuses'][-1] != self.status):
                res['statuses'].append(self.status)
            if self.status in ('Failed', 'Canceled') and fresh:
                break
            # Distance to the reference path, while moving:
            if (fresh and self.ref_path and len(self.ref_path) > 1 and
                    self.status == 'Running' and self.gt[3] > 0.05):
                res['max_dev'] = max(
                    res['max_dev'], dist_to_polyline(self.gt[:2], self.ref_path))
            if self.status == 'ReachedGoal' and fresh:
                t_reached = t_reached or time.time()
                # Let it settle:
                if time.time() - t_reached > 2.0:
                    break
            else:
                t_reached = None
        res['time'] = time.time() - t_sent
        res['final_status'] = self.status
        res['pos_err'] = math.hypot(self.gt[0] - g[0], self.gt[1] - g[1])
        res['yaw_err'] = abs(math.degrees(angle_diff(self.gt[2], math.radians(g[2]))))
        return res

    def test_navigation_battery(self):
        # Wait for the whole system to be up: simulator, localization, planner
        # (it publishes its footprint once initialized) and follower:
        self.assertTrue(
            self.spin_until(
                lambda: self.gt is not None and self.robot_shape is not None and
                self.status is not None and self.localized(),
                STARTUP_TIMEOUT),
            'Timeout waiting for the navigation system to be ready and localized')

        # The planner footprint must cover the simulated vehicle chassis:
        self.spin_until(lambda: self.chassis is not None, 5.0)
        if self.chassis is not None:
            for p in self.chassis:
                self.assertTrue(
                    point_in_polygon(p, self.robot_shape),
                    f'Chassis vertex {p} outside of the planner footprint '
                    f'{self.robot_shape}')

        self.spin_until(lambda: False, 3.0)
        crit = CRITERIA[ROBOT]
        failures = []
        reached = 0
        for i, g in enumerate(GOALS):
            r = self.navigate_to(g)
            print(f'[{ROBOT}] goal #{i} {g}: {r}', flush=True)
            if r['final_status'] == 'ReachedGoal':
                reached += 1
                if r['pos_err'] > crit['max_pos_err'] or r['yaw_err'] > crit['max_yaw_err']:
                    failures.append(f'goal #{i} reported as reached, but it is not: {r}')
            elif r['final_status'] not in ('Failed', 'Canceled'):
                failures.append(f'goal #{i}: navigation did not end in time: {r}')
            if r['max_dev'] > MAX_PATH_DEVIATION:
                failures.append(f'goal #{i}: robot too far from its path: {r}')
        if reached < crit['min_reached']:
            failures.append(f'only {reached} of {len(GOALS)} goals reached '
                            f'(required: {crit["min_reached"]})')

        self.assertEqual(self.collisions, 0, 'The robot collided')
        self.assertFalse(failures, '\n'.join(failures))
