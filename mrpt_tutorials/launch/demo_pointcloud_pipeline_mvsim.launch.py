# ROS 2 launch file for example in mrpt_tutorials
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Launch for mrpt_pointcloud_pipeline:
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'scan_topic_name': '/laser1, /laser2',
            'points_topic_name': '/camera1_points',
            'time_window': '0.20',
            'show_gui': 'True',
        }.items()
    )

    mvsim_pkg_share_dir = get_package_share_directory('mvsim')
    # Finding the launch file
    launch_file_name = 'demo_jackal.launch.py'
    mvsim_launch_file_path = os.path.join(
        mvsim_pkg_share_dir, 'mvsim_tutorial', launch_file_name)

    # Check if the launch file exists
    if not os.path.isfile(mvsim_launch_file_path):
        raise Exception(
            f"Launch file '{mvsim_launch_file_path}' does not exist!")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mvsim_launch_file_path)
        ),
        pointcloud_pipeline_launch
    ])
