import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    lidar_topic_name_arg = DeclareLaunchArgument(
        'lidar_topic_name',
        default_value='/laser1, /laser2'
    )
    points_topic_name_arg = DeclareLaunchArgument(
        'points_topic_name',
        default_value='/ouster/points, /camera1_points'
    )
    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='True'
    )
    time_window_arg = DeclareLaunchArgument(
        'time_window',
        default_value='0.20'
    )
    filter_yaml_file_arg = DeclareLaunchArgument(
        'filter_yaml_file',
        default_value=os.path.join(os.path.dirname(
            __file__), 'local-obstacles-decimation-filter.yaml')
    )
    filter_output_layer_name_arg = DeclareLaunchArgument(
        'filter_output_layer_name',
        default_value='output'
    )
    filter_output_topic_arg = DeclareLaunchArgument(
        'filter_output_topic_name',
        default_value='/local_map_pointcloud'
    )
    frameid_reference_arg = DeclareLaunchArgument(
        'frameid_reference',
        default_value='odom'
    )
    frameid_robot_arg = DeclareLaunchArgument(
        'frameid_robot',
        default_value='base_link'
    )

    # Node: Local obstacles builder
    mrpt_pointcloud_pipeline_node = Node(
        package='mrpt_pointcloud_pipeline',
        executable='mrpt_pointcloud_pipeline_node',
        name='mrpt_pointcloud_pipeline_node',
        output='screen',
        parameters=[
            {'source_topics_2dscan': LaunchConfiguration('lidar_topic_name')},
            {'source_topics_pointclouds': LaunchConfiguration(
                'points_topic_name')},
            {'show_gui': LaunchConfiguration('show_gui')},
            {'filter_yaml_file': LaunchConfiguration('filter_yaml_file')},
            {'filter_output_layer_name': LaunchConfiguration(
                'filter_output_layer_name')},
            {'time_window': LaunchConfiguration('time_window')},
            {'topic_local_map_pointcloud': LaunchConfiguration(
                'filter_output_topic_name')},
            {'frameid_reference': LaunchConfiguration('frameid_reference')},
            {'frameid_robot': LaunchConfiguration('frameid_robot')},
        ],
    )

    mvsim_pkg_share_dir = get_package_share_directory('mvsim')
    # Finding the launch file
    launch_file_name = 'demo_jackal.launch.py'
    mvsim_launch_file_path = os.path.join(
        mvsim_pkg_share_dir, 'mvsim_tutorial', launch_file_name)

    # Check if the launch file exists
    if not os.path.isfile(mvsim_launch_file_path):
        raise Exception(
            f"Launch file '{mvsim_launch_file_path}' does not exist!")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mvsim_launch_file_path)
        ),
        lidar_topic_name_arg,
        points_topic_name_arg,
        show_gui_arg,
        time_window_arg,
        filter_yaml_file_arg,
        filter_output_layer_name_arg,
        filter_output_topic_arg,
        frameid_reference_arg,
        frameid_robot_arg,
        mrpt_pointcloud_pipeline_node
    ])
