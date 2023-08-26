
# ROS2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    pfLocDir = get_package_share_directory("mrpt_pf_localization")
    pfTutorialsDir = get_package_share_directory("mrpt_tutorials")
    print('pfLocDir       : ' + pfLocDir)
    print('pfTutorialsDir : ' + pfTutorialsDir)

    # args that can be set from the command line or a default will be used
    mrpt_map_config_file_launch_arg = DeclareLaunchArgument(
        "mrpt_map_config_file", default_value=TextSubstitution(
            text=os.path.join(pfLocDir, 'params', 'map-occgrid2d.ini')))

    mrpt_simplemap_file_launch_arg = DeclareLaunchArgument(
        "mrpt_simplemap_file", default_value=TextSubstitution(
            text=os.path.join(pfTutorialsDir, 'maps', 'gh25_real_top_laser.simplemap')))
    
    defaultParamsFile = os.path.join(pfLocDir, 'params', 'default.config.yaml')

    mvsim_node = Node(
        package='mrpt_pf_localization',
        executable='mrpt_pf_localization_node',
        name='mrpt_pf_localization',
        output='screen',
        parameters=[
            defaultParamsFile,
            {
                "mrpt_map_config_file": LaunchConfiguration('mrpt_map_config_file'),
                "mrpt_simplemap_file": LaunchConfiguration('mrpt_simplemap_file'),
            }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [os.path.join(pfTutorialsDir, 'rviz', 'demo_localization.rviz')]]
    )

    return LaunchDescription([
        mrpt_map_config_file_launch_arg,
        mrpt_simplemap_file_launch_arg,
        mvsim_node,
        rviz2_node
    ])
