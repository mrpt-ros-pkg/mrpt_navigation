# ROS 2 launch file for example in mrpt_tutorials
#
# Same as demo_astar_rnav_gridmap.launch.py, but uses the newer
# mrpt_trajectory_follower (mpp::TrajectoryFollower) instead of
# mrpt_reactivenav2d to drive the robot along the A* planned path.
#
# Launch arguments:
#  - robot: 'ackermann' (car-like, default) or 'diffdrive' (Jackal-like).
#    Each one selects its own mvsim world, PTG set (kinematics + footprint),
#    and a small file of robot-specific follower overrides. The planner and
#    follower parameter files are shared.
#  - speed_limit: initial follower speed limit [m/s] (<=0: platform max).
#    Can be changed at run time, e.g.:
#      ros2 param set /mrpt_trajectory_follower speed_limit 2.0
#  - max_computation_time: max A* computation time per plan request [s]
#  - anytime_first_path_time, continuous_planning, replan_period, ...: planner
#    goal manager settings (see PLANNER_GOAL_MANAGER_ARGS below)
#    (<=0: value of the planner parameters file).
#  - use_rviz: False for headless runs.
#  - use_sim_time: True (default) to run all nodes on the simulation clock
#    published by mvsim, so they stay consistent with sensor timestamps even
#    if the simulation runs slower than real time.
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            GroupAction, OpaqueFunction)
from launch_ros.actions import SetParameter
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# Per-robot settings:
ROBOTS = {
    'ackermann': {
        'world': 'demo_world2.world.xml',
        'ptg_ini': ('mrpt_tutorials', 'params', 'ptgs_mvsim_ackermann.ini'),
        'scan_topics': '/laser1, /laser2',
        'follower_overrides': 'follower-overrides-ackermann.yaml',
    },
    'diffdrive': {
        'world': 'demo_world2_diffdrive.world.xml',
        'ptg_ini': ('mrpt_tps_astar_planner', 'configs', 'ini', 'ptgs_jackal.ini'),
        'scan_topics': '/laser1',
        'follower_overrides': 'follower-overrides-diffdrive.yaml',
    },
}


# Planner goal manager settings (see mrpt_tps_astar_planner/README.md), with the
# planner defaults: (name, default, description)
PLANNER_GOAL_MANAGER_ARGS = [
    ('anytime_first_path_time', '2.5',
     'Planner: publish the best partial path after this time [s] (<=0: only full paths)'),
    ('anytime_min_progress', '1.0',
     'Planner: minimum progress toward the goal [m] of a partial path'),
    ('anytime_max_partial_length', '10.0',
     'Planner: partial paths are cut to this length [m] (<=0: no limit)'),
    ('continuous_planning', 'true',
     'Planner: keep replanning while the robot drives'),
    ('replan_period', '1.0',
     'Planner: period of the replanning cycles [s]'),
    ('stitch_lookahead_time', '3.0',
     'Planner: the search goes on from this far ahead [s] of the robot on its path'),
    ('start_lookahead_time', '3.0',
     'Planner: new searches from a moving robot start this far ahead [s] on its path'),
    ('topic_follower_status_ex_sub', '/mrpt_trajectory_follower/status_ex',
     'Planner: follower status with path ids (empty: use the plain status topic)'),
    ('no_progress_timeout', '60.0',
     'Planner: a goal fails if the robot does not get closer to it for this long [s]'),
    ('no_progress_warn_fraction', '0.5',
     'Planner: the status turns to STALLED (a warning) after this fraction of no_progress_timeout without progress (<=0: never)'),
    ('max_planning_restarts', '0',
     'Planner: if >0, searches restarted in a row for a goal before giving up'),
]


def launch_setup(context, *args, **kwargs):
    tutsDir = get_package_share_directory("mrpt_tutorials")
    astarDir = get_package_share_directory("mrpt_tps_astar_planner")

    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise ValueError(
            f"Unknown robot '{robot}', valid values: {list(ROBOTS.keys())}")
    cfg = ROBOTS[robot]

    ptg_ini_file = os.path.join(get_package_share_directory(
        cfg['ptg_ini'][0]), *cfg['ptg_ini'][1:])

    mrpt_astar_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            astarDir, 'launch', 'tps_astar_planner.launch.py')]),
        launch_arguments={
            'topic_goal_sub': '/goal_pose',
            'show_gui': 'False',
            'topic_obstacles_gridmap_sub': '/mrpt_map/map_gridmap',
            'topic_static_maps': '/mrpt_map/map_gridmap',
            # Also sensed obstacles, so replans avoid unmapped ones:
            'topic_obstacles_sub': ', '.join(
                t for t in ['/local_map_pointcloud',
                            LaunchConfiguration('planner_extra_obstacles_topic').perform(context)]
                if t),
            'topic_wp_seq_pub': '/waypoints',
            'ptg_ini': ptg_ini_file,
            'max_computation_time': LaunchConfiguration('max_computation_time'),
            **{name: LaunchConfiguration(name) for name, _, _ in PLANNER_GOAL_MANAGER_ARGS},
            'planner_parameters': LaunchConfiguration('planner_parameters'),
        }.items()
    )

    mrpt_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_map_server'), 'launch',
            'mrpt_map_server.launch.py')]),
        launch_arguments={
            'map_yaml_file': os.path.join(tutsDir, 'maps', 'demo_world2.yaml'),
        }.items()
    )

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            {
                "world_file": os.path.join(tutsDir, 'mvsim', cfg['world']),
                "do_fake_localization": False,
                "headless": True,
            }]
    )

    # Launch for pf_localization:
    pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch',
            'localization.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'log_level_core': 'INFO',
            'topic_sensors_2d_scan': cfg['scan_topics'],
            # Start localized at the robot pose in the mvsim world file:
            'pf_params_overrides_file': LaunchConfiguration('pf_params_overrides_file'),
            'topic_sensors_point_clouds': '',
            'base_link_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
        }.items()
    )

    # Launch for mrpt_pointcloud_pipeline (recent obstacles, odom frame):
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            'use_composable': 'false',
            'log_level': 'INFO',
            'scan_topic_name': cfg['scan_topics'],
            'points_topic_name': '',
            'filter_output_topic_name': '/local_map_pointcloud',
            'time_window': '0.20',
            'show_gui': 'False',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    # Launch for mrpt_trajectory_follower:
    trajectory_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_trajectory_follower'), 'launch',
            'trajectory_follower.launch.py')]),
        launch_arguments={
            'frame_id_map': 'map',
            'frame_id_robot': 'base_link',
            'topic_path_sub': '/waypoints_path',  # published by mrpt_tps_astar_planner
            'topic_obstacles_sub': '/local_map_pointcloud',
            'topic_odom_sub': '/odom',
            'topic_cmd_vel_pub': '/cmd_vel',
            'ptg_ini': ptg_ini_file,
            'follower_parameters_overrides': os.path.join(
                tutsDir, 'params', cfg['follower_overrides']),
            'speed_limit': LaunchConfiguration('speed_limit'),
            'replan_on_failure': 'True',
            'log_level': 'INFO',
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=[
                '-d', [os.path.join(tutsDir, 'rviz2', 'gridmap.rviz')]]
    )
    # mvsim is the clock source; all other nodes may use its clock:
    return [
        mvsim_node,
        GroupAction([
            SetParameter(name='use_sim_time',
                         value=LaunchConfiguration('use_sim_time')),
            mrpt_map_launch,
            mrpt_astar_planner_launch,
            pf_localization_launch,
            pointcloud_pipeline_launch,
            trajectory_follower_launch,
            rviz2_node,
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='ackermann',
            description='Simulated robot: ' + ', '.join(ROBOTS.keys())),
        DeclareLaunchArgument(
            'max_computation_time', default_value='0.0',
            description='Max A* computation time per plan request [s] (<=0: planner file value)'),
        DeclareLaunchArgument(
            'speed_limit', default_value='0.0',
            description='Initial follower speed limit [m/s] (<=0: platform max)'),
        DeclareLaunchArgument(
            'planner_parameters',
            default_value=os.path.join(
                get_package_share_directory('mrpt_tps_astar_planner'), 'configs', 'params',
                'planner-params.yaml'),
            description='A* planner parameters file'),
        DeclareLaunchArgument(
            'planner_extra_obstacles_topic', default_value='',
            description='Additional obstacle points topic for the planner only (e.g. for tests)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='True',
            description='Whether to launch RViz2 (False for headless runs)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Run the nodes on the simulation clock from mvsim'),
        DeclareLaunchArgument(
            'pf_params_overrides_file',
            default_value=os.path.join(
                get_package_share_directory('mrpt_tutorials'), 'params',
                'pf-initial-pose-demo_world2.yaml'),
            description='mrpt_pf_localization parameter overrides (the initial pose must '
            'match the robot pose in the mvsim world)'),
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, default, desc in PLANNER_GOAL_MANAGER_ARGS],
        OpaqueFunction(function=launch_setup),
    ])
