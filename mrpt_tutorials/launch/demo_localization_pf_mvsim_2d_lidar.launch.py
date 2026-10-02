# ROS 2 launch file for example in mrpt_tutorials
#
# All nodes except mvsim run on the simulation clock published by mvsim
# (launch argument use_sim_time, default True), so they stay consistent with
# sensor timestamps even if the simulation runs slower than real time.
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")
    # print('tutsDir       : ' + tutsDir)

    # Launch for pf_localization:
    pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch',
            'localization.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'log_level_core': 'INFO',
            'topic_sensors_2d_scan': '/laser1, /laser2',
            # Start localized at the robot pose in the mvsim world file:
            'pf_params_overrides_file': os.path.join(
                tutsDir, 'params', 'pf-initial-pose-demo_world2.yaml'),
            'topic_sensors_point_clouds': '',

            # For robots with wheels odometry, use:     'base_link'-> 'odom'      -> 'map'
            # For systems without wheels odometry, use: 'base_link'-> 'base_link' -> 'map'
            'base_link_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
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
            os.path.join(tutsDir, 'params', 'mvsim_ros2_params.yaml'),
            {
                "world_file": os.path.join(tutsDir, 'mvsim', 'demo_world2.world.xml'),
            }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=[
                '-d', [os.path.join(tutsDir, 'rviz2', 'gridmap.rviz')]]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='True',
            description='Whether to launch RViz2 (False for headless runs)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Run the nodes on the simulation clock from mvsim'),
        mvsim_node,
        GroupAction([
            SetParameter(name='use_sim_time',
                         value=LaunchConfiguration('use_sim_time')),
            pf_localization_launch,
            rviz2_node,
            mrpt_map_launch,
        ]),
    ])
