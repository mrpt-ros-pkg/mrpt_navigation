import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=get_package_share_directory(
                'mvsim') + '/mvsim_tutorial/mvsim_demo_1robot.world.xml'
        ),
        launch_ros.actions.Node(
            package='mrpt_pointcloud_pipeline',
            executable='mrpt_pointcloud_pipeline_node',
            name='mrpt_pointcloud_pipeline_node',
            output='screen',
            parameters=[
                {
                    'source_topics_2dscan': 'laser1,laser2'
                },
                {
                    'show_gui': 'false'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='mvsim',
            executable='mvsim_node',
            name='mvsim_simulator',
            output='screen',
            parameters=[
                {
                    'world_file': launch.substitutions.LaunchConfiguration('world_file')
                },
                {
                    'do_fake_localization': 'true'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz'
        ),
        launch_ros.actions.Node(
            package='mrpt_reactivenav2d',
            executable='mrpt_reactivenav2d_node',
            name='mrpt_reactivenav2d_node',
            output='screen',
            parameters=[
                {
                    'cfg_file_reactive': get_package_share_directory('mrpt_reactivenav2d') + '/tutorial/reactive2d_config.ini'
                },
                {
                    'topic_robot_shape': '/chassis_polygon'
                }
            ]
        )
    ])
    return ld

