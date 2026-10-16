# Integration test: replanning for a fast robot, end-to-end in simulation
# (mvsim, headless, diffdrive robot at its 1.5 m/s top speed).
#
# New searches from a moving robot start ahead of it on the path it follows
# (planner start_lookahead_time), keeping that path up to there, so it does
# not get past the start of the new path before it is published. Checks that:
#  - A new goal given while driving at full speed is taken without stopping or
#    leaving the path (the follower switches to the new path keeping its
#    speed).
#  - An obstacle that appears close ahead on the path while driving at full
#    speed (seen by the planner only) is avoided, without collisions,
#    blockages or leaving the path, and the goal is reached.
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
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mrpt_nav_interfaces.msg import ReferencePath
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String

STATUS_TOPIC = '/mrpt_trajectory_follower/status'
ROBOT_SHAPE_TOPIC = '/mrpt_tps_astar_planner_node/robot_shape'
REFERENCE_PATH_TOPIC = '/waypoints_reference'
VIRTUAL_OBSTACLES_TOPIC = '/test_virtual_obstacles'

STARTUP_TIMEOUT = 300.0     # [s]
GOAL_TIMEOUT = 120.0        # [s]
CRUISE_SPEED = 1.3          # [m/s] "full speed" (platform max: 1.5 m/s)

# Goals (x [m], y [m], yaw [deg]), map frame. The robot starts at (30, 30).
FIRST_GOAL = (9.20, 17.95, -165.0)  # far, to the south-west
NEW_GOAL = (22.0, 38.0, 90.0)       # to the north, given while driving
LAST_GOAL = (40.0, 20.0, 0.0)       # far, to the east

# Minimum robot speed [m/s] in the seconds after a new goal, while driving at
# full speed (it must not stop to switch paths):
MIN_SPEED_ON_NEW_GOAL = 0.5
NEW_GOAL_SPEED_WINDOW = 3.0  # [s]

# Virtual obstacle: a square of this half-size [m], this far ahead [m] of the
# robot along its path (within the part of the path already committed):
OBSTACLE_HALF_SIZE = 0.4
OBSTACLE_AHEAD = 4.5
MIN_CLEARANCE = 0.25  # [m] robot center to the obstacle square


# Ground truth distance [m] to the goal accepted as reached. The follower stops
# using the localized pose, whose error (up to MAX_LOC_ERROR in the battery) is
# not part of what these tests check, so this must not be tighter than that.
GOAL_GT_TOLERANCE = 1.0

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
                'speed_limit': '0.0',  # platform max speed
                'use_rviz': 'False',
                'planner_extra_obstacles_topic': VIRTUAL_OBSTACLES_TOPIC,
                'pf_params_overrides_file': os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), 'pf-test-overrides.yaml'),
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestFastRobotReplanning(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fast_robot_replanning_test')
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.status = None
        cls.statuses = []
        cls.collisions = 0
        cls.robot_shape = None
        cls.ref = None  # latest ReferencePath poses
        cls.obstacle = None
        cls.last_obstacle_pub = 0.0

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

        def on_reference(m):
            cls.ref = [(q.pose.position.x, q.pose.position.y) for q in m.path.poses]

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        n.create_subscription(
            PolygonStamped, ROBOT_SHAPE_TOPIC,
            lambda m: setattr(cls, 'robot_shape', m.polygon.points), latched)
        n.create_subscription(ReferencePath, REFERENCE_PATH_TOPIC, on_reference, 10)
        cls.pub_goal = n.create_publisher(PoseStamped, '/goal_pose', 10)
        cls.pub_obstacles = n.create_publisher(PointCloud2, VIRTUAL_OBSTACLES_TOPIC, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.obstacle and time.time() - self.last_obstacle_pub > 0.2:
            self.publish_obstacle()

    def spin_until(self, predicate, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_once()
            if predicate():
                return True
        return False

    def send_goal(self, g):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = g[0]
        msg.pose.position.y = g[1]
        msg.pose.orientation.z = math.sin(math.radians(g[2]) / 2)
        msg.pose.orientation.w = math.cos(math.radians(g[2]) / 2)
        self.pub_goal.publish(msg)

    def publish_obstacle(self):
        cx, cy = self.obstacle
        h = OBSTACLE_HALF_SIZE
        pts = []
        for i in range(9):
            t = -h + 2 * h * i / 8
            pts += [(cx + t, cy - h, 0.2), (cx + t, cy + h, 0.2),
                    (cx - h, cy + t, 0.2), (cx + h, cy + t, 0.2)]
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.node.get_clock().now().to_msg()
        self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, pts))
        self.last_obstacle_pub = time.time()

    def clearance_to_obstacle(self):
        cx, cy = self.obstacle
        dx = max(abs(self.gt[0] - cx) - OBSTACLE_HALF_SIZE, 0.0)
        dy = max(abs(self.gt[1] - cy) - OBSTACLE_HALF_SIZE, 0.0)
        return math.hypot(dx, dy)

    def point_ahead(self, dist):
        path = self.ref
        i0 = min(range(len(path)),
                 key=lambda i: math.hypot(path[i][0] - self.gt[0], path[i][1] - self.gt[1]))
        s = 0.0
        for i in range(i0 + 1, len(path)):
            s += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if s >= dist:
                return path[i]
        return None

    def wait_reached(self, g):
        def reached():
            return self.status == 'ReachedGoal' and math.hypot(
                self.gt[0] - g[0], self.gt[1] - g[1]) < GOAL_GT_TOLERANCE
        return self.spin_until(reached, GOAL_TIMEOUT)

    def test_fast_robot_replanning(self):
        ok = self.spin_until(
            lambda: self.robot_shape is not None and self.gt is not None and
            self.status is not None, STARTUP_TIMEOUT)
        self.assertTrue(ok, 'navigation nodes not ready')
        t0 = time.time()
        while time.time() - t0 < 10.0:  # let the localization converge
            self.spin_once()

        # 1) A new goal while driving at full speed:
        self.send_goal(FIRST_GOAL)
        self.assertTrue(self.spin_until(lambda: self.gt[3] > CRUISE_SPEED, 30.0),
                        'the robot did not reach full speed')
        print(f'Full speed ({self.gt[3]:.2f} m/s) at {self.gt[:2]}: new goal', flush=True)
        self.statuses.clear()
        self.send_goal(NEW_GOAL)
        t_new = time.time()
        min_speed = float('inf')
        while time.time() - t_new < NEW_GOAL_SPEED_WINDOW:
            self.spin_once()
            min_speed = min(min_speed, self.gt[3])
        print(f'Min speed after the new goal: {min_speed:.2f} m/s, statuses: {self.statuses}',
              flush=True)
        self.assertGreater(min_speed, MIN_SPEED_ON_NEW_GOAL, 'the robot stopped on a new goal')
        self.assertTrue(self.wait_reached(NEW_GOAL), f'new goal not reached: {self.statuses}')
        for bad in ('OffPathExceeded', 'Blocked', 'Canceled', 'Failed'):
            self.assertNotIn(bad, self.statuses)
        self.assertEqual(self.collisions, 0)

        # 2) An obstacle appears close ahead while driving at full speed:
        self.statuses.clear()
        self.__class__.ref = None  # (the path to the previous goal)
        self.send_goal(LAST_GOAL)
        self.assertTrue(
            self.spin_until(lambda: self.ref is not None and self.gt[3] > CRUISE_SPEED, 30.0),
            'no new path, or the robot did not reach full speed')
        p = self.point_ahead(OBSTACLE_AHEAD)
        self.assertIsNotNone(p, 'path too short to place the obstacle')
        self.__class__.obstacle = p
        print(f'Virtual obstacle at {p}, robot at {self.gt[:2]} ({self.gt[3]:.2f} m/s)',
              flush=True)
        self.publish_obstacle()
        min_clearance = float('inf')

        def reached():
            nonlocal min_clearance
            min_clearance = min(min_clearance, self.clearance_to_obstacle())
            return self.status == 'ReachedGoal' and math.hypot(
                self.gt[0] - LAST_GOAL[0], self.gt[1] - LAST_GOAL[1]) < GOAL_GT_TOLERANCE
        ok = self.spin_until(reached, GOAL_TIMEOUT)
        self.__class__.obstacle = None
        print(f'Min clearance to the obstacle: {min_clearance:.2f} m, statuses: {self.statuses}',
              flush=True)
        self.assertTrue(ok, f'goal not reached: {self.statuses}')
        self.assertGreater(min_clearance, MIN_CLEARANCE, 'the robot drove into the obstacle')
        for bad in ('OffPathExceeded', 'Blocked', 'Failed'):
            self.assertNotIn(bad, self.statuses)
        self.assertEqual(self.collisions, 0)
