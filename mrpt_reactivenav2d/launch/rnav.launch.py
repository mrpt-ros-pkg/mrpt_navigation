# ROS 2 launch file for mrpt_reactivenav2d, intended to be included in user's
# launch files.
#
# See the docs on the configurable launch arguments for this file in:
# https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_reactivenav2d
#
# For ready-to-launch demos, see: https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_tutorials
#

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from ament_index_python import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import os


def generate_launch_description():
    # myPkgDir = get_package_share_directory("mrpt_reactivenav2d")
    # print('myPkgDir: ' + myPkgDir)

    default_rnav_cfg_file = os.path.join(
        get_package_share_directory('mrpt_reactivenav2d'),
        'params', 'reactive2d_default.ini')

    rnav_cfg_file_arg = DeclareLaunchArgument(
        'rnav_cfg_file',
        default_value=default_rnav_cfg_file
    )
    topic_robot_shape_arg = DeclareLaunchArgument(
        'topic_robot_shape',
        default_value=''
    )
    topic_obstacles_arg = DeclareLaunchArgument(
        'topic_obstacles',
        default_value='/local_map_pointcloud'
    )
    topic_reactive_nav_goal_arg = DeclareLaunchArgument(
        'topic_reactive_nav_goal',
        default_value='/goal_pose'
    )
    topic_reactive_nav_waypoint_sequence_arg = DeclareLaunchArgument(
        'topic_wp_seq',
        default_value='/rnav_waypoint_sequence'
    )
    nav_period_arg = DeclareLaunchArgument(
        'nav_period',
        default_value='0.20'
    )
    frameid_reference_arg = DeclareLaunchArgument(
        'frameid_reference',
        default_value='map'
    )
    frameid_robot_arg = DeclareLaunchArgument(
        'frameid_robot',
        default_value='base_link'
    )
    save_nav_log_arg = DeclareLaunchArgument(
        'save_nav_log',
        default_value='False'
    )
    topic_cmd_vel_arg = DeclareLaunchArgument(
        'topic_cmd_vel',
        default_value='/cmd_vel'
    )
    pure_pursuit_mode_launch_arg = DeclareLaunchArgument(
        "pure_pursuit_mode",
        default_value=TextSubstitution(text=str("False")),
        description="If enabled, no obstacle avoidance will be attempted"
    )

    log_level_launch_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("INFO")),
        description="Logging level"
    )

    emit_shutdown_action = Shutdown(reason='launch is shutting down')

    # remappings={('/cmd_vel', LaunchConfiguration('cmd_vel_out'))},

    node_rnav2d_launch = Node(
        package='mrpt_reactivenav2d',
        executable='mrpt_reactivenav2d_node',
        name='mrpt_reactivenav2d_node',
        output='screen',
        parameters=[
            {
                'cfg_file_reactive': LaunchConfiguration('rnav_cfg_file'),
                'topic_robot_shape': LaunchConfiguration('topic_robot_shape'),
                'topic_obstacles': LaunchConfiguration('topic_obstacles'),
                'topic_reactive_nav_goal': LaunchConfiguration('topic_reactive_nav_goal'),
                'topic_wp_seq':  LaunchConfiguration('topic_wp_seq'),
                'nav_period': LaunchConfiguration('nav_period'),
                'frameid_reference': LaunchConfiguration('frameid_reference'),
                'frameid_robot': LaunchConfiguration('frameid_robot'),
                'save_nav_log': LaunchConfiguration('save_nav_log'),
                'topic_cmd_vel': LaunchConfiguration('topic_cmd_vel'),
                'pure_pursuit_mode': LaunchConfiguration('pure_pursuit_mode'),
            }
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        on_exit=[emit_shutdown_action]
    )

    ld = LaunchDescription([
        rnav_cfg_file_arg,
        log_level_launch_arg,
        topic_robot_shape_arg,
        topic_obstacles_arg,
        topic_reactive_nav_goal_arg,
        topic_reactive_nav_waypoint_sequence_arg,
        nav_period_arg,
        frameid_reference_arg,
        frameid_robot_arg,
        save_nav_log_arg,
        topic_cmd_vel_arg,
        pure_pursuit_mode_launch_arg,
        node_rnav2d_launch,
    ])

    # Namespace to avoid clash launch argument names with the parent scope:
    return LaunchDescription([GroupAction(
        actions=[
            PushRosNamespace(namespace='rnav'),
            ld
        ])])
