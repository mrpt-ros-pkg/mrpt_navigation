# ROS 2 launch file for example in mrpt_tutorials
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")
    # print('tutsDir       : ' + tutsDir)

    mrpt_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_map_server'), 'launch',
            'mrpt_map_server.launch.py')]),
        launch_arguments={
            'map_yaml_file': os.path.join(tutsDir, 'maps', 'demo_world2.yaml'),
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [os.path.join(tutsDir, 'rviz2', 'gridmap.rviz')]]
    )
    return LaunchDescription([
        mrpt_map_launch,
        rviz2_node
    ])
