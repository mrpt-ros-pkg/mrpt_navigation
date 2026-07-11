# ROS 2 launch for the mrpt_trajectory_follower node.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mrpt_trajectory_follower')

    default_params = PathJoinSubstitution(
        [pkg_share, 'configs', 'params', 'follower-params.yaml'])

    args = [
        DeclareLaunchArgument('frame_id_map', default_value='map'),
        DeclareLaunchArgument('frame_id_robot', default_value='base_link'),
        DeclareLaunchArgument('topic_path_sub', default_value='/waypoints_path'),
        DeclareLaunchArgument('topic_obstacles_sub', default_value=''),
        DeclareLaunchArgument('topic_odom_sub', default_value='/odom'),
        DeclareLaunchArgument('topic_cmd_vel_pub', default_value='/cmd_vel'),
        DeclareLaunchArgument('ptg_ini', default_value=''),
        DeclareLaunchArgument('follower_parameters', default_value=default_params),
        # Obstacle-cloud conditioning (for a raw 3D lidar source):
        DeclareLaunchArgument('robot_radius', default_value='0.0'),
        DeclareLaunchArgument('obstacle_z_min', default_value='-1000000.0'),
        DeclareLaunchArgument('obstacle_z_max', default_value='1000000.0'),
        DeclareLaunchArgument('self_filter_radius', default_value='0.0'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    node = Node(
        package='mrpt_trajectory_follower',
        executable='mrpt_trajectory_follower_node',
        name='mrpt_trajectory_follower',
        output='screen',
        parameters=[{
            'frame_id_map': LaunchConfiguration('frame_id_map'),
            'frame_id_robot': LaunchConfiguration('frame_id_robot'),
            'topic_path_sub': LaunchConfiguration('topic_path_sub'),
            'topic_obstacles_sub': LaunchConfiguration('topic_obstacles_sub'),
            'topic_odom_sub': LaunchConfiguration('topic_odom_sub'),
            'topic_cmd_vel_pub': LaunchConfiguration('topic_cmd_vel_pub'),
            'ptg_ini': LaunchConfiguration('ptg_ini'),
            'follower_parameters': LaunchConfiguration('follower_parameters'),
            'robot_radius': ParameterValue(
                LaunchConfiguration('robot_radius'), value_type=float),
            'obstacle_z_min': ParameterValue(
                LaunchConfiguration('obstacle_z_min'), value_type=float),
            'obstacle_z_max': ParameterValue(
                LaunchConfiguration('obstacle_z_max'), value_type=float),
            'self_filter_radius': ParameterValue(
                LaunchConfiguration('self_filter_radius'), value_type=float),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([*args, node])
