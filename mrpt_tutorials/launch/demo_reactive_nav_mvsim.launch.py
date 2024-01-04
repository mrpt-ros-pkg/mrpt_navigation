import os
import sys

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")

    default_world_file = os.path.join(
        get_package_share_directory('mvsim'),
        'mvsim_tutorial', 'demo_warehouse.world.xml')

    default_rnav_cfg_file = os.path.join(
        get_package_share_directory('mrpt_reactivenav2d'),
        'tutorial', 'reactive2d_config.ini')

    arg_world_file_launch = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file
    )

    filter_yaml_file_arg = DeclareLaunchArgument(
        'filter_yaml_file',
        default_value=os.path.join(get_package_share_directory(
            'mrpt_pointcloud_pipeline'), 'launch', 'local-obstacles-decimation-filter.yaml')
    )

    node_pointcloud_pipeline_launch = Node(
        package='mrpt_pointcloud_pipeline',
        executable='mrpt_pointcloud_pipeline_node',
        name='mrpt_pointcloud_pipeline_node',
        output='screen',
        parameters=[
            {
                # 2D lidar sources:
                'source_topics_2dscan': 'scanner1',
                # 3D lidar sources:
                'source_topics_pointclouds': 'lidar1_points',
                'filter_yaml_file': LaunchConfiguration('filter_yaml_file'),
            },
            {
                'show_gui': False
            }
        ]
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

    node_rnav2d_launch = Node(
        package='mrpt_reactivenav2d',
        executable='mrpt_reactivenav2d_node',
        name='mrpt_reactivenav2d_node',
        output='screen',
        parameters=[
            {
                'cfg_file_reactive': default_rnav_cfg_file,
            },
            {
                'topic_robot_shape': '/chassis_polygon'
            }
        ]
    )

    return LaunchDescription([
        arg_world_file_launch,
        filter_yaml_file_arg,
        node_pointcloud_pipeline_launch,
        node_mvsim_launch,
        node_rviz2_launch,
        node_rnav2d_launch
    ])
