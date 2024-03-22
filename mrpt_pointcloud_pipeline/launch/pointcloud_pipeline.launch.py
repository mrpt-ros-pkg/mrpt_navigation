# ROS 2 launch file for mrpt_pointcloud_pipeline, intended to be included in user's
# launch files.
#
# See the docs on the configurable launch arguments for this file in:
# https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pointcloud_pipeline
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
    myPkgDir = get_package_share_directory("mrpt_pointcloud_pipeline")
    # print('myPkgDir       : ' + myPkgDir)

    lidar_topic_name_arg = DeclareLaunchArgument(
        'scan_topic_name',
        default_value=''  # /scan, /laser1, etc.
    )
    points_topic_name_arg = DeclareLaunchArgument(
        'points_topic_name',
        default_value=''  # '/ouster/points', etc.
    )
    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='True'
    )
    time_window_arg = DeclareLaunchArgument(
        'time_window',
        default_value='0.20'
    )
    one_observation_per_topic_arg = DeclareLaunchArgument(
        'one_observation_per_topic',
        default_value='false'
    )
    pipeline_yaml_file_arg = DeclareLaunchArgument(
        'pipeline_yaml_file',
        default_value=os.path.join(
            myPkgDir, 'params', 'point-cloud-pipeline.yaml')
    )
    filter_output_layer_name_arg = DeclareLaunchArgument(
        'filter_output_layer_name',
        default_value='output',
        description='The mp2p_icp metric_map_t layer name(s) to be published. Comma-separated list of more than one.'
    )
    filter_output_topic_arg = DeclareLaunchArgument(
        'filter_output_topic_name',
        default_value='/local_map_pointcloud',
        description='The topic name to publish the output layer map(s). Comma-separated list of more than one, then the number must match that of filter_output_layer_name.'
    )
    frameid_reference_arg = DeclareLaunchArgument(
        'frameid_reference',
        default_value='odom'
    )
    frameid_robot_arg = DeclareLaunchArgument(
        'frameid_robot',
        default_value='base_link'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("INFO")),
        description="Logging level"
    )

    emit_shutdown_action = Shutdown(reason='launch is shutting down')

    mrpt_pointcloud_pipeline_node = Node(
        package='mrpt_pointcloud_pipeline',
        executable='mrpt_pointcloud_pipeline_node',
        name='mrpt_pointcloud_pipeline_node',
        output='screen',
        parameters=[
            {'source_topics_2d_scans': LaunchConfiguration('scan_topic_name')},
            {'source_topics_pointclouds': LaunchConfiguration(
                'points_topic_name')},
            {'show_gui': LaunchConfiguration('show_gui')},
            {'pipeline_yaml_file': LaunchConfiguration('pipeline_yaml_file')},
            {'filter_output_layer_name': LaunchConfiguration(
                'filter_output_layer_name')},
            {'time_window': LaunchConfiguration('time_window')},
            {'topic_local_map_pointcloud': LaunchConfiguration(
                'filter_output_topic_name')},
            {'frameid_reference': LaunchConfiguration(
                'frameid_reference')},
            {'frameid_robot': LaunchConfiguration('frameid_robot')},
            {'one_observation_per_topic': LaunchConfiguration(
                'one_observation_per_topic')},
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        on_exit=[emit_shutdown_action]
    )

    ld = LaunchDescription([
        lidar_topic_name_arg,
        points_topic_name_arg,
        show_gui_arg,
        time_window_arg,
        pipeline_yaml_file_arg,
        filter_output_layer_name_arg,
        filter_output_topic_arg,
        frameid_reference_arg,
        frameid_robot_arg,
        log_level_launch_arg,
        one_observation_per_topic_arg,
        mrpt_pointcloud_pipeline_node,
    ])

    # Namespace to avoid clash launch argument names with the parent scope:
    return LaunchDescription([GroupAction(
        actions=[
            PushRosNamespace(namespace='pc_pipeline'),
            ld
        ])])
