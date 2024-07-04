# ROS 2 launch file for mrpt_map_server
#
# See the docs on the configurable launch arguments for this file in:
# https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server#template-ros-2-launch-files
#

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # mrpt_map_pkg_dir = get_package_share_directory('mrpt_map_server')


    # Format 1 in docs:
    mm_file_arg = DeclareLaunchArgument(
        'mm_file',
        default_value=''
    )
    # (legacy) Format 2 in docs:
    map_yaml_file_arg = DeclareLaunchArgument(
        'map_yaml_file', default_value=''
    )
    # Format 3 in docs:
    mrpt_metricmap_file_arg = DeclareLaunchArgument(
        'mrpt_metricmap_file', default_value=''
    )
    
    # Others:
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map'
    )
    pub_mm_topic_arg = DeclareLaunchArgument(
        'pub_mm_topic',
        default_value='mrpt_map'
    )

    # Node: Map server
    mrpt_map_server_node = Node(
        package='mrpt_map_server',
        executable='map_server_node',
        name='map_server_node',
        output='screen',
        parameters=[
            {'map_yaml_file': LaunchConfiguration('map_yaml_file')},
            {'mm_file': LaunchConfiguration('mm_file')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'mrpt_metricmap_file': LaunchConfiguration('mrpt_metricmap_file')},
            {'pub_mm_topic': LaunchConfiguration('pub_mm_topic')},
        ],
    )

    return LaunchDescription([
        map_yaml_file_arg,
        mm_file_arg,
        mrpt_metricmap_file_arg,
        frame_id_arg,
        pub_mm_topic_arg,
        mrpt_map_server_node
    ])
