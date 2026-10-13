# ROS 2 launch file for example in mrpt_tutorials
#
# Same as demo_astar_trajectory_follower_gridmap.launch.py (ackermann robot),
# but in a "street" world with parking bays at different angles (perpendicular,
# 60 and 120 degrees, parallel), a planter, a kiosk and bollards, to exercise
# the A* planner. Parked cars are polygonal obstacles. Try RViz "2D Goal Pose"
# goals inside the bays, e.g. (7.25, 3.5, 90 deg), (30.0, 4.0, 60 deg),
# (15.0, 24.1, 0 deg) or (36.5, 24.1, 0 deg).
#
# Launch arguments:
#  - speed_limit: initial follower speed limit [m/s] (<=0: platform max).
#  - max_computation_time: max A* computation time per plan request [s]
#    (<=0: value of the planner parameters file).
#  - use_rviz: False for headless runs.
#  - use_sim_time: True (default) to run all nodes on the simulation clock
#    published by mvsim.
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

# Robot settings (same car-like vehicle as the ackermann robot of the other demo):
CFG = {
    'world': 'demo_street.world.xml',
    'map_yaml': 'street.yaml',
    'ptg_ini': ('mrpt_tutorials', 'params', 'ptgs_mvsim_ackermann.ini'),
    'scan_topics': '/laser1, /laser2',
    'follower_overrides': 'follower-overrides-ackermann.yaml',
}


def launch_setup(context, *args, **kwargs):
    tutsDir = get_package_share_directory("mrpt_tutorials")
    astarDir = get_package_share_directory("mrpt_tps_astar_planner")

    cfg = CFG

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
            'topic_obstacles_sub': '/local_map_pointcloud',
            'topic_wp_seq_pub': '/waypoints',
            'ptg_ini': ptg_ini_file,
            'max_computation_time': LaunchConfiguration('max_computation_time'),
        }.items()
    )

    mrpt_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_map_server'), 'launch',
            'mrpt_map_server.launch.py')]),
        launch_arguments={
            'map_yaml_file': os.path.join(tutsDir, 'maps', cfg['map_yaml']),
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
            'max_computation_time', default_value='0.0',
            description='Max A* computation time per plan request [s] (<=0: planner file value)'),
        DeclareLaunchArgument(
            'speed_limit', default_value='0.0',
            description='Initial follower speed limit [m/s] (<=0: platform max)'),
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
                'pf-initial-pose-demo_street.yaml'),
            description='mrpt_pf_localization parameter overrides (the initial pose must '
            'match the robot pose in the mvsim world)'),
        OpaqueFunction(function=launch_setup),
    ])
