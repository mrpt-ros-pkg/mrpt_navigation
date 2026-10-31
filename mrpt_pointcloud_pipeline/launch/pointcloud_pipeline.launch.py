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
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import os


def generate_launch_description():
    myPkgDir = get_package_share_directory("mrpt_pointcloud_pipeline")
    # print('myPkgDir       : ' + myPkgDir)

    use_composable = LaunchConfiguration('use_composable')

    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='false',
        description='If true, load as a composable node into container_name instead of a standalone node'
    )

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
    memory_layers_arg = DeclareLaunchArgument(
        'memory_layers',
        default_value='',
        description='Output layer(s) that also report obstacles remembered inside memory_region (comma-separated). Empty: disabled.'
    )
    memory_region_arg = DeclareLaunchArgument(
        'memory_region',
        default_value='',
        description='Region where the sensors are blind, as "x_min, y_min, x_max, y_max" in the robot frame [m]'
    )
    memory_self_box_arg = DeclareLaunchArgument(
        'memory_self_box',
        default_value='',
        description='Points inside this box are never remembered (they may be the robot itself), as "x_min, y_min, x_max, y_max" in the robot frame [m]'
    )
    memory_max_age_arg = DeclareLaunchArgument(
        'memory_max_age',
        default_value='10.0',
        description='Remembered obstacles are forgotten after this long without being observed [s]'
    )
    confirm_radius_arg = DeclareLaunchArgument(
        'confirm_radius',
        default_value='0.0',
        description='If >0, output points are only published if the previous candidate points built from other observations had a point this close (in the reference frame), so one-off artifacts of a single scan are dropped [m]'
    )
    confirm_max_age_arg = DeclareLaunchArgument(
        'confirm_max_age',
        default_value='0.5',
        description='Past candidate points older than this are not used to confirm points [s]. Must be larger than time_window, unless one_observation_per_topic'
    )
    tf_max_wait_arg = DeclareLaunchArgument(
        'tf_max_wait',
        default_value='0.5',
        description='How long an observation waits for the robot pose at its time before being dropped [s]'
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

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of the composable node container (required when use_composable:=true)',
    )

    emit_shutdown_action = Shutdown(reason='launch is shutting down')

    mrpt_pointcloud_pipeline_node = Node(
        condition=UnlessCondition(use_composable),
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
            {'memory_layers': LaunchConfiguration('memory_layers')},
            {'memory_region': LaunchConfiguration('memory_region')},
            {'memory_self_box': LaunchConfiguration('memory_self_box')},
            {'memory_max_age': LaunchConfiguration('memory_max_age')},
            {'confirm_radius': LaunchConfiguration('confirm_radius')},
            {'confirm_max_age': LaunchConfiguration('confirm_max_age')},
            {'tf_max_wait': LaunchConfiguration('tf_max_wait')},
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        on_exit=[emit_shutdown_action]
    )

    composable_mrpt_pointcloud_pipeline = LoadComposableNodes(
        condition=IfCondition(use_composable),
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='mrpt_pointcloud_pipeline',
                name='mrpt_pointcloud_pipeline_component',
                plugin='LocalObstaclesNode',
                parameters=[
                    {'source_topics_2d_scans': LaunchConfiguration('scan_topic_name')},
                    {'source_topics_pointclouds': LaunchConfiguration('points_topic_name')},
                    {'show_gui': LaunchConfiguration('show_gui')},
                    {'pipeline_yaml_file': LaunchConfiguration('pipeline_yaml_file')},
                    {'filter_output_layer_name': LaunchConfiguration('filter_output_layer_name')},
                    {'time_window': LaunchConfiguration('time_window')},
                    {'topic_local_map_pointcloud': LaunchConfiguration('filter_output_topic_name')},
                    {'frameid_reference': LaunchConfiguration('frameid_reference')},
                    {'frameid_robot': LaunchConfiguration('frameid_robot')},
                    {'one_observation_per_topic': LaunchConfiguration('one_observation_per_topic')},
                    {'memory_layers': LaunchConfiguration('memory_layers')},
                    {'memory_region': LaunchConfiguration('memory_region')},
                    {'memory_self_box': LaunchConfiguration('memory_self_box')},
                    {'memory_max_age': LaunchConfiguration('memory_max_age')},
                    {'confirm_radius': LaunchConfiguration('confirm_radius')},
                    {'confirm_max_age': LaunchConfiguration('confirm_max_age')},
                    {'tf_max_wait': LaunchConfiguration('tf_max_wait')},
                ],
            )
        ]
    )

    ld = LaunchDescription([
        use_composable_arg,
        container_name_arg,
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
        memory_layers_arg,
        memory_region_arg,
        memory_self_box_arg,
        memory_max_age_arg,
        confirm_radius_arg,
        confirm_max_age_arg,
        tf_max_wait_arg,
        mrpt_pointcloud_pipeline_node,
        composable_mrpt_pointcloud_pipeline,
    ])

    # Namespace to avoid clash launch argument names with the parent scope:
    return LaunchDescription([GroupAction(
        actions=[
            PushRosNamespace(namespace='pc_pipeline'),
            ld
        ])])
