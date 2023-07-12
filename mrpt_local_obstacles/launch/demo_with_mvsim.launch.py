import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
        
    lidar_topic_name_arg = DeclareLaunchArgument(
        'lidar_topic_name', 
        default_value='laser1,laser2'
    )
    points_topic_name_arg = DeclareLaunchArgument(
        'points_topic_name', 
        default_value='/fake_obstacles'
    )
    show_gui_arg = DeclareLaunchArgument(
        'show_gui', 
        default_value='True'
    )
    time_window_arg = DeclareLaunchArgument(
        'time_window', 
        default_value='0.2'
    )
    filter_yaml_file_arg = DeclareLaunchArgument(
        'filter_yaml_file', 
        default_value=''
    )
    filter_output_layer_name_arg = DeclareLaunchArgument(
        'filter_output_layer_name', 
        default_value=''
    )

    # Node: Local obstacles builder
    mrpt_local_obstacles_node = Node(
        package='mrpt_local_obstacles',
        executable='mrpt_local_obstacles_node',
        name='mrpt_local_obstacles_node',
        output='screen',
        parameters=[
            {'source_topics_2dscan': LaunchConfiguration('lidar_topic_name')},
            {'source_topics_pointclouds': LaunchConfiguration('points_topic_name')},
            {'show_gui': LaunchConfiguration('show_gui')},
            {'filter_yaml_file': LaunchConfiguration('filter_yaml_file')},
            {'filter_output_layer_name': LaunchConfiguration('filter_output_layer_name')},
            {'time_window': LaunchConfiguration('time_window')},
            {'topic_local_map_pointcloud': '/local_map_pointcloud'},
        ],
    )

    mvsim_node = Node(
        package='mvsim',
        executable ='mvsim',
        name='mvsim_node',
        output='screen',
        parameters=[],
    )

    return LaunchDescription([
        lidar_topic_name_arg,
        points_topic_name_arg,
        show_gui_arg,
        time_window_arg,
        filter_yaml_file_arg,
        filter_output_layer_name_arg,
        mrpt_local_obstacles_node
        #mvsim_node
    ])
