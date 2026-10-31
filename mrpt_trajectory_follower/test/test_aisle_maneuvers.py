# Integration test: the trajectory follower node driving a compact car-like
# robot in simulation (mvsim, headless), between narrow aisles.
#
# The robot starts inside an aisle, facing out of it. It drives two
# geometric legs, generated from its actual pose as a waypoint generator would:
#  1. Forward out of the aisle and a quarter-ellipse turn into the crossing
#     aisle, away from the next aisle.
#  2. Reverse straight along the crossing aisle and a reverse 90 deg arc into
#     the next aisle, then a short straight inside it.
# The obstacle cloud (the box outlines) is published by the test itself.
#
# Checks: each leg ends in ReachedGoal in time, the simulator reports no
# collision, the footprint keeps a minimum clearance to the boxes, and the
# robot leaves the reverse arc on the in-aisle straight, not wide of it.
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
from geometry_msgs.msg import PoseStamped
from launch_ros.actions import Node
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String

TEST_DIR = os.path.dirname(os.path.abspath(__file__))

STATUS_TOPIC = '/mrpt_trajectory_follower_node/status'
PATH_TOPIC = '/waypoints_path'
OBSTACLES_TOPIC = '/test_obstacles'

# Geometry of aisles.world.xml:
AISLE_PITCH = 1.8      # [m] between aisle centerlines
ROW_END_X = 1.5        # [m] open end of the aisles
MOUTH_DEPTH = 0.43     # [m] reference point inside the aisle, for entering
EXIT_ANCHOR_X = 0.93   # [m] reference point inside the aisle, for leaving
FOOTPRINT = [(-0.35, 0.43), (1.0, 0.43), (1.4, 0.30), (1.4, -0.30),
             (1.0, -0.43), (-0.35, -0.43)]

# Leg shapes:
ELLIPSE_FORWARD_FACTOR = 1.1  # forward semi-axis, in aisle pitches
TAIL_STRAIGHT = 0.45          # [m] after each turn
ALIGN_STRAIGHT = 0.6          # [m] reverse arc ends this far out of the mouth
EXTRA_DEPTH = 0.4             # [m] goal past the mouth

# Acceptance criteria:
STARTUP_TIMEOUT = 60.0     # [s]
LEG_TIMEOUT = 45.0         # [s]
MIN_CLEARANCE = 0.02       # [m] footprint to boxes (the reverse arc nominally keeps 0.25 m;
                           # under CPU load the follower lags up to ~0.2 m entering it)
MAX_IN_AISLE_CROSS = 0.25  # [m] off the in-aisle straight of the reverse leg (timing-sensitive on
                           # CI; MIN_CLEARANCE and the collision check are the safety bounds)


def box_outlines():
    """Obstacle points (x, y) on the outline of every box in the world."""
    centers = []
    for row in range(4):
        for i in range(15):
            centers.append((-5.75 + 0.5 * i, -0.9 + 1.8 * row))
    for i in range(20):
        centers.append((4.2, -4.75 + 0.5 * i))
    pts = []
    for cx, cy in centers:
        for k in range(9):
            u = -0.2 + 0.05 * k
            pts += [(cx - 0.2, cy + u), (cx + 0.2, cy + u),
                    (cx + u, cy - 0.2), (cx + u, cy + 0.2)]
    return pts


OBSTACLES = box_outlines()


def footprint_clearance(x, y, yaw):
    """Minimum distance from the footprint polygon at (x,y,yaw) to the boxes."""
    c, s = math.cos(yaw), math.sin(yaw)
    best = float('inf')
    n = len(FOOTPRINT)
    for ox, oy in OBSTACLES:
        if abs(ox - x) > 2.5 or abs(oy - y) > 2.5:
            continue
        lx = c * (ox - x) + s * (oy - y)
        ly = -s * (ox - x) + c * (oy - y)
        inside = True
        for i in range(n):
            ax, ay = FOOTPRINT[i]
            bx, by = FOOTPRINT[(i + 1) % n]
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((lx - ax) * dx + (ly - ay) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(lx - ax - t * dx, ly - ay - t * dy))
            inside = inside and (dx * (ly - ay) - dy * (lx - ax)) <= 0
        if inside:
            return 0.0
    return best


def dist_to_segment(p, a, b):
    sx, sy = b[0] - a[0], b[1] - a[1]
    l2 = sx * sx + sy * sy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * sx + (p[1] - a[1]) * sy) / l2))
    return math.hypot(p[0] - a[0] - t * sx, p[1] - a[1] - t * sy)


def exit_leg(x0, y0):
    """Forward out of the aisle (heading +x), quarter-ellipse turn to the
    right (away from the next aisle), and a short straight. Knots as
    (x, y, body heading)."""
    pts = [(x0, y0, 0.0)]
    xa = max(x0, EXIT_ANCHOR_X)
    if xa > x0 + 0.05:
        pts.append((xa, y0, 0.0))
    a = ELLIPSE_FORWARD_FACTOR * AISLE_PITCH
    b = AISLE_PITCH
    for i in range(1, 13):
        t = (math.pi / 2) * i / 12
        pts.append((xa + a * math.sin(t), y0 - b * (1 - math.cos(t)),
                    -math.atan2(b * math.sin(t), a * math.cos(t))))
    xe, ye, _ = pts[-1]
    for i in range(1, 4):
        pts.append((xe, ye - TAIL_STRAIGHT * i / 3, -math.pi / 2))
    return pts


def entry_leg(xs, ys, aisle_y):
    """Reverse along +y (body heading -y) on the crossing aisle line where the
    exit leg ends, reverse 90 deg arc into the aisle centered at aisle_y
    (body heading +x at its end), and a reverse straight into it. The line,
    not the actual start pose, sets the arc, so how precisely the exit leg
    stopped doesn't change the maneuver. Returns the knots and the in-aisle
    straight segment."""
    xl = EXIT_ANCHOR_X + ELLIPSE_FORWARD_FACTOR * AISLE_PITCH
    xt = ROW_END_X - MOUTH_DEPTH + ALIGN_STRAIGHT
    r = xl - xt
    ya = aisle_y - r
    pts = [(xs, ys, -math.pi / 2)]
    for i in range(1, 7):
        pts.append((xl, ys + (ya - ys) * i / 6, -math.pi / 2))
    # Arc centered at (xt, ya), traveled counterclockwise from (xs, ya):
    for i in range(1, 13):
        t = (math.pi / 2) * i / 12
        pts.append((xt + r * math.cos(t), ya + r * math.sin(t), -math.pi / 2 + t))
    length = ALIGN_STRAIGHT + TAIL_STRAIGHT + EXTRA_DEPTH
    for i in range(1, 4):
        pts.append((xt - length * i / 3, aisle_y, 0.0))
    return pts, ((xt, aisle_y), (xt - length, aisle_y))


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
            'obstacle_z_min': -1.0e6,
            'obstacle_z_max': 1.0e6,
            'self_filter_radius': 0.0,
        }])
    return launch.LaunchDescription([
        mvsim, follower, launch_testing.actions.ReadyToTest()])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class AisleManeuvers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'aisle_maneuvers_test',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        n = cls.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.gt = None
        cls.status = None
        cls.status_count = 0
        cls.collisions = 0
        cls.trace = []

        def on_gt(m):
            p = m.pose.pose
            cls.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
            cls.trace.append(cls.gt)

        def on_status(m):
            cls.status = m.data
            cls.status_count += 1

        def on_collision(m):
            if m.data:
                cls.collisions += 1

        n.create_subscription(Odometry, '/base_pose_ground_truth', on_gt, 10)
        n.create_subscription(String, STATUS_TOPIC, on_status, latched)
        n.create_subscription(Bool, '/collision', on_collision, 100)
        cls.pub_path = n.create_publisher(Path, PATH_TOPIC, latched)
        cls.pub_obs = n.create_publisher(PointCloud2, OBSTACLES_TOPIC, 10)
        cls.obs_xyz = [(x, y, 0.5) for x, y in OBSTACLES]
        n.create_timer(0.1, cls.publish_obstacles)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_obstacles(cls):
        h = Header()
        h.frame_id = 'map'
        h.stamp = cls.node.get_clock().now().to_msg()
        cls.pub_obs.publish(point_cloud2.create_cloud_xyz32(h, cls.obs_xyz))

    def spin_for(self, secs):
        t_end = time.time() + secs
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def publish_path(self, knots):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for x, y, yaw in knots:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def drive(self, knots):
        """Publishes a leg and waits for it to end. Returns (final status,
        duration, ground-truth trace)."""
        count0 = self.status_count
        self.trace.clear()
        self.publish_path(knots)
        t0 = time.time()
        started = False
        while time.time() - t0 < LEG_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.status_count > count0 and self.status == 'Running':
                started = True
            if started and self.status not in ('Running', None):
                break
        return self.status, time.time() - t0, list(self.trace)

    def test_aisle_to_aisle(self):
        t0 = time.time()
        while self.gt is None or self.status is None:
            self.assertLess(time.time() - t0, STARTUP_TIMEOUT, 'Nodes did not start')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.spin_for(2.0)

        min_clearance = float('inf')

        # Leg 1: out of the aisle.
        x, y, _ = self.gt
        status, dur, trace = self.drive(exit_leg(x, y))
        self.node.get_logger().info(f'Exit leg: {status} in {dur:.1f} s')
        self.assertEqual(status, 'ReachedGoal', 'Exit leg did not reach its goal')
        min_clearance = min([min_clearance] + [footprint_clearance(*p) for p in trace])

        # Leg 2: into the next aisle, from the actual pose.
        self.spin_for(1.0)
        x, y, _ = self.gt
        knots, (a, b) = entry_leg(x, y, AISLE_PITCH)
        status, dur, trace = self.drive(knots)
        self.node.get_logger().info(f'Entry leg: {status} in {dur:.1f} s')
        self.assertEqual(status, 'ReachedGoal', 'Entry leg did not reach its goal')
        min_clearance = min([min_clearance] + [footprint_clearance(*p) for p in trace])
        in_aisle = [p for p in trace if p[0] < a[0]]
        self.assertTrue(in_aisle, 'The robot never got into the aisle')
        max_cross = max(dist_to_segment(p, a, b) for p in in_aisle)

        self.node.get_logger().info(
            f'min clearance {min_clearance:.3f} m, in-aisle cross-track {max_cross:.3f} m, '
            f'collisions {self.collisions}')
        self.assertEqual(self.collisions, 0, 'The robot collided')
        self.assertGreater(min_clearance, MIN_CLEARANCE, 'Too close to the boxes')
        self.assertLess(max_cross, MAX_IN_AISLE_CROSS, 'Left the reverse arc wide of the aisle')
