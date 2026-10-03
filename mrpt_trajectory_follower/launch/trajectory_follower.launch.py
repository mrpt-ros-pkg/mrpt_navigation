# ROS 2 launch for the mrpt_trajectory_follower node.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mrpt_trajectory_follower')

    default_params = PathJoinSubstitution(
        [pkg_share, 'configs', 'params', 'follower-params.yaml'])

    use_composable = LaunchConfiguration('use_composable')

    args = [
        DeclareLaunchArgument(
            'use_composable', default_value='false',
            description='If true, load as a composable node into container_name instead of a standalone node'),
        DeclareLaunchArgument(
            'container_name', default_value='',
            description='Name of the composable node container (required when use_composable:=true)'),
        DeclareLaunchArgument('frame_id_map', default_value='map'),
        DeclareLaunchArgument('frame_id_robot', default_value='base_link'),
        DeclareLaunchArgument('topic_path_sub', default_value='/waypoints_path'),
        DeclareLaunchArgument('topic_obstacles_sub', default_value=''),
        DeclareLaunchArgument('topic_odom_sub', default_value='/odom'),
        DeclareLaunchArgument('topic_cmd_vel_pub', default_value='/cmd_vel'),
        DeclareLaunchArgument('ptg_ini', default_value=''),
        DeclareLaunchArgument('follower_parameters', default_value=default_params),
        # Optional YAML file whose keys override those in follower_parameters
        # (e.g. robot-specific kinematic limits):
        DeclareLaunchArgument('follower_parameters_overrides', default_value=''),
        # Run-time speed limit [m/s] (<=0: use max_speed from the params file):
        DeclareLaunchArgument('speed_limit', default_value='0.0'),
        # Obstacle-cloud conditioning (for a raw 3D lidar source):
        DeclareLaunchArgument('robot_radius', default_value='0.0'),
        DeclareLaunchArgument('obstacle_z_min', default_value='-1000000.0'),
        DeclareLaunchArgument('obstacle_z_max', default_value='1000000.0'),
        DeclareLaunchArgument('self_filter_radius', default_value='0.0'),
        # Last-resort collision guard on every command (needs topic_obstacles_sub):
        DeclareLaunchArgument('collision_guard', default_value='True'),
        # On leaving the path or getting blocked, request a new plan to the
        # same goal from this service (mrpt_nav_interfaces/MakePlanTo):
        DeclareLaunchArgument('replan_on_failure', default_value='False'),
        DeclareLaunchArgument('max_replan_attempts', default_value='3'),
        DeclareLaunchArgument(
            'planner_service', default_value='/mrpt_tps_astar_planner_node/make_plan_to'),
        # Footprint published by the planner, to check both use the same one
        # (empty: no check):
        DeclareLaunchArgument(
            'topic_robot_shape_sub', default_value='/mrpt_tps_astar_planner_node/robot_shape'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    node_parameters = [{
        'frame_id_map': LaunchConfiguration('frame_id_map'),
        'frame_id_robot': LaunchConfiguration('frame_id_robot'),
        'topic_path_sub': LaunchConfiguration('topic_path_sub'),
        'topic_obstacles_sub': LaunchConfiguration('topic_obstacles_sub'),
        'topic_odom_sub': LaunchConfiguration('topic_odom_sub'),
        'topic_cmd_vel_pub': LaunchConfiguration('topic_cmd_vel_pub'),
        'ptg_ini': LaunchConfiguration('ptg_ini'),
        'follower_parameters': LaunchConfiguration('follower_parameters'),
        'follower_parameters_overrides': LaunchConfiguration(
            'follower_parameters_overrides'),
        'speed_limit': ParameterValue(
            LaunchConfiguration('speed_limit'), value_type=float),
        'robot_radius': ParameterValue(
            LaunchConfiguration('robot_radius'), value_type=float),
        'obstacle_z_min': ParameterValue(
            LaunchConfiguration('obstacle_z_min'), value_type=float),
        'obstacle_z_max': ParameterValue(
            LaunchConfiguration('obstacle_z_max'), value_type=float),
        'self_filter_radius': ParameterValue(
            LaunchConfiguration('self_filter_radius'), value_type=float),
        'collision_guard': ParameterValue(
            LaunchConfiguration('collision_guard'), value_type=bool),
        'replan_on_failure': ParameterValue(
            LaunchConfiguration('replan_on_failure'), value_type=bool),
        'max_replan_attempts': ParameterValue(
            LaunchConfiguration('max_replan_attempts'), value_type=int),
        'planner_service': LaunchConfiguration('planner_service'),
        'topic_robot_shape_sub': LaunchConfiguration('topic_robot_shape_sub'),
    }]

    node = Node(
        condition=UnlessCondition(use_composable),
        package='mrpt_trajectory_follower',
        executable='mrpt_trajectory_follower_node',
        name='mrpt_trajectory_follower',
        output='screen',
        parameters=node_parameters,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    composable_node = LoadComposableNodes(
        condition=IfCondition(use_composable),
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='mrpt_trajectory_follower',
                name='mrpt_trajectory_follower',
                plugin='mrpt_trajectory_follower::TrajectoryFollowerNode',
                parameters=node_parameters,
            )
        ]
    )

    return LaunchDescription([*args, node, composable_node])
