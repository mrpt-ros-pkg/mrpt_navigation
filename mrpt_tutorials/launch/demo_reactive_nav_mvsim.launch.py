# ROS 2 launch file for example in mrpt_tutorials
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")

    default_world_file = os.path.join(
        get_package_share_directory('mvsim'),
        'mvsim_tutorial', 'demo_warehouse.world.xml')

    arg_world_file_launch = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file
    )

    # Launch for mrpt_pointcloud_pipeline:
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'scan_topic_name': '/scanner1',
            'points_topic_name': '/lidar1_points',
            'filter_output_topic_name': '/local_map_pointcloud',
            'time_window': '0.20',
            'show_gui': 'True',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    node_mvsim_launch = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim_simulator',
        output='screen',
        parameters=[
            {
                'world_file': LaunchConfiguration('world_file')
            },
            {
                # This allows running without SLAM/Particle filter (Disable when using them!)
                'do_fake_localization': True
            }
        ]
    )

    node_rviz2_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', [os.path.join(tutsDir, 'rviz2', 'rnav_demo.rviz')]]
    )

    # Launch for mrpt_reactivenav2d:
    node_rnav2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_reactivenav2d'), 'launch',
            'rnav.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            # 'save_nav_log': 'True',
            'frameid_robot': 'base_link',
            'frameid_reference': 'map',
        }.items()
    )

    return LaunchDescription([
        arg_world_file_launch,
        pointcloud_pipeline_launch,
        node_mvsim_launch,
        node_rviz2_launch,
        node_rnav2d_launch
    ])
