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
from launch.actions import (DeclareLaunchArgument,
                            EmitEvent, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import os


def generate_launch_description():
    pfLocDir = get_package_share_directory("mrpt_pf_localization")
    # print('pfLocDir       : ' + pfLocDir)

    pf_params_file_launch_arg = DeclareLaunchArgument(
        "pf_params_file", default_value=TextSubstitution(
            text=os.path.join(pfLocDir, 'params', 'default.config.yaml')))

    relocalization_params_file_launch_arg = DeclareLaunchArgument(
        "relocalization_params_file", default_value=TextSubstitution(
            text=os.path.join(pfLocDir, 'params', 'default-relocalization-pipeline.yaml')))

    pf_log_level_launch_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("INFO")),
        description="Logging level for the ROS node and the PF core class"
    )

    pf_log_level_core_launch_arg = DeclareLaunchArgument(
        "log_level_core",
        default_value=TextSubstitution(text=str("INFO")),
        description="Logging level for PF core C++ class (DEBUG|INFO|WARN|ERROR)"
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

    topic_gnss_args = DeclareLaunchArgument(
        "topic_gnss",
        default_value='/gps',
        description="Topic to subscribe for NavSatFix msgs for georeferenced initialization"
    )

    base_link_frame_id_arg = DeclareLaunchArgument(
        "base_link_frame_id",
        default_value='base_link',
        description="frame_id for the vehicle base_link"
    )
    odom_frame_id_arg = DeclareLaunchArgument(
        "odom_frame_id",
        default_value='odom',
        description="frame_id for the vehicle odom"
    )
    global_frame_id_arg = DeclareLaunchArgument(
        "global_frame_id",
        default_value='map',
        description="frame_id for the vehicle global frame (typ: 'map')"
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
                "topic_sensors_2d_scan": LaunchConfiguration('topic_sensors_2d_scan'),
                "topic_sensors_point_clouds": LaunchConfiguration('topic_sensors_point_clouds'),
                "topic_gnss": LaunchConfiguration('topic_gnss'),
                "relocalization_params_file": LaunchConfiguration('relocalization_params_file'),
                "gui_enable": LaunchConfiguration('gui_enable'),
                "log_level_core": LaunchConfiguration('log_level_core'),
                "base_link_frame_id": LaunchConfiguration('base_link_frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id'),
                "global_frame_id": LaunchConfiguration('global_frame_id'),
            }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')]
    )

    ld = LaunchDescription([
        pf_log_level_launch_arg,
        pf_log_level_core_launch_arg,
        relocalization_params_file_launch_arg,
        pf_params_file_launch_arg,
        topic_sensors_2d_scan_arg,
        topic_sensors_point_clouds_arg,
        topic_gnss_args,
        gui_enable_arg,
        base_link_frame_id_arg,
        odom_frame_id_arg,
        global_frame_id_arg,
        pf_localization_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=pf_localization_node,
                on_exit=[
                    LogInfo(msg=('mrpt_pf_localization ended')),
                    EmitEvent(event=Shutdown(
                        reason='mrpt_pf_localization ended'))
                ])
        )
    ])

    # Namespace to avoid clash launch argument names with the parent scope:
    return LaunchDescription([GroupAction(
        actions=[
            PushRosNamespace(namespace='pf_localization'),
            ld
        ])])
