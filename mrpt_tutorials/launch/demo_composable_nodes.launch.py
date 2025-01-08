# ROS 2 launch file for example in mrpt_tutorials
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")
    # print('tutsDir       : ' + tutsDir)

    use_composable = LaunchConfiguration('use_composable')

    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='false',
    )

    container = ComposableNodeContainer(
        condition = IfCondition(use_composable),
        name='demo_composable_container',
        package = 'rclcpp_components',
        executable = 'component_container',
        namespace = '',
        output = 'screen',
    )

    mrpt_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_map_server'), 'launch',
            'mrpt_map_server.launch.py')]),
        launch_arguments={
            'map_yaml_file': os.path.join(tutsDir, 'maps', 'demo_world2.yaml'),
        }.items()
    )

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            # os.path.join(tutsDir, 'params', 'mvsim_ros2_params.yaml'),
            {
                "world_file": os.path.join(tutsDir, 'mvsim', 'demo_world2.world.xml'),
                "do_fake_localization": True,
                "headless": True,
            }]
    )

    # Launch for mrpt_pointcloud_pipeline:
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            #
            'use_composable': LaunchConfiguration('use_composable'),
            'container_name': 'demo_composable_container',
            #
            'log_level': 'INFO',
            'scan_topic_name': '/laser1, /laser2',
            'points_topic_name': '/lidar1_points',
            'filter_output_topic_name': '/local_map_pointcloud',
            'time_window': '0.20',
            'show_gui': 'False',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    # Launch for pf_localization:
    pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch',
            'localization.launch.py')]),
        launch_arguments={
            #
            'use_composable': LaunchConfiguration('use_composable'),
            'container_name': 'demo_composable_container',
            #
            'log_level': 'INFO',
            'log_level_core': 'INFO',
            'topic_sensors_2d_scan': '/laser1',
            'topic_sensors_point_clouds': '',

            # For robots with wheels odometry, use:     'base_link'-> 'odom'      -> 'map'
            # For systems without wheels odometry, use: 'base_link'-> 'base_link' -> 'map'
            'base_link_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
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
        use_composable_arg,
        container,
        mrpt_map_launch,
        pf_localization_launch,
        mvsim_node,
        pointcloud_pipeline_launch,
        rviz2_node,
    ])
