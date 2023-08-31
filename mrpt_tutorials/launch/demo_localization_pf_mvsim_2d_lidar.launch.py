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

    # Launch for pf_localization:
    pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch'),
            'localization.launch.py']),
        launch_arguments={
            'mrpt_metricmap_file': os.path.join(tutsDir, '', ''),
        }.items()
    )

    return LaunchDescription([
        pf_localization_launch
    ])
