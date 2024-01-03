# ROS 2 launch file for mrpt_map
#
# See the docs on the configurable launch arguments for this file in:
# https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map#template-ros-2-launch-files
#

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # mrpt_map_pkg_dir = get_package_share_directory('mrpt_map')

    map_yaml_file_arg = DeclareLaunchArgument(
        'map_yaml_file', default_value=''
    )
    mm_file_arg = DeclareLaunchArgument(
        'mm_file',
        default_value=''
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map'
    )
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='1.0'
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='False'
    )

    # Node: Local obstacles builder
    mrpt_map_server_node = Node(
        package='mrpt_map',
        executable='map_server_node',
        name='map_server_node',
        output='screen',
        parameters=[
            {'map_yaml_file': LaunchConfiguration('map_yaml_file')},
            {'mm_file': LaunchConfiguration('mm_file')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'frequency': LaunchConfiguration('frequency')},
            {'debug': LaunchConfiguration('debug')},
            {'pub_map_ros': '/map'},
            {'pub_metadata': '/map_metadata'},
            {'service_map': 'static_map'}
        ],
    )

    return LaunchDescription([
        map_yaml_file_arg,
        mm_file_arg,
        frame_id_arg,
        frequency_arg,
        debug_arg,
        mrpt_map_server_node
    ])
