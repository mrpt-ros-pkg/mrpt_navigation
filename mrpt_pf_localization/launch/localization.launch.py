# ROS 2 launch file for mrpt_pf_localization, intended to be included in user's
# launch files.
#
# See the docs on the configurable launch arguments for this file in:
# https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pf_localization#template-ros-2-launch-files
#
# For ready-to-launch demos, see: https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_tutorials
#

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    pfLocDir = get_package_share_directory("mrpt_pf_localization")
    # print('pfLocDir       : ' + pfLocDir)

    # args that can be set from the command line or a default will be used
    mrpt_map_config_file_launch_arg = DeclareLaunchArgument(
        "mrpt_map_config_file", default_value=TextSubstitution(
            text=os.path.join(pfLocDir, 'params', 'map-occgrid2d.ini')))

    pf_params_file_launch_arg = DeclareLaunchArgument(
        "pf_params_file", default_value=TextSubstitution(
            text=os.path.join(pfLocDir, 'params', 'default.config.yaml')))

    pf_log_level_launch_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("INFO")),
        description="Logging level"
    )

    topic_sensors_2d_scan_arg = DeclareLaunchArgument(
        "topic_sensors_2d_scan",
        default_value='',
        description="Comma-separated list of topics to subscribe for LaserScan msgs as sensor inputs"
    )
    topic_sensors_point_clouds_arg = DeclareLaunchArgument(
        "topic_sensors_point_clouds",
        default_value='',
        description="Comma-separated list of topics to subscribe for PointCloud2 msgs as sensor inputs"
    )

    gui_enable_arg = DeclareLaunchArgument(
        "gui_enable",
        default_value='False',
        description="Whether to show a custom UI with details on the PF status"
    )

    pf_localization_node = Node(
        package='mrpt_pf_localization',
        executable='mrpt_pf_localization_node',
        name='mrpt_pf_localization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('pf_params_file'),
            {
                "mrpt_map_config_file": LaunchConfiguration('mrpt_map_config_file'),
                "topic_sensors_2d_scan": LaunchConfiguration('topic_sensors_2d_scan'),
                "topic_sensors_point_clouds": LaunchConfiguration('topic_sensors_point_clouds'),
                "gui_enable": LaunchConfiguration('gui_enable'),
            }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        mrpt_map_config_file_launch_arg,
        pf_log_level_launch_arg,
        pf_params_file_launch_arg,
        topic_sensors_2d_scan_arg,
        topic_sensors_point_clouds_arg,
        gui_enable_arg,
        pf_localization_node,
    ])
