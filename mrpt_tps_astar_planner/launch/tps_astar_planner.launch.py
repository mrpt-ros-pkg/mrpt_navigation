from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory('mrpt_tps_astar_planner')

    # Declare launch arguments
    topic_goal_sub = DeclareLaunchArgument(
        'topic_goal_sub', default_value='/goal_pose',
        description='Goal subscription topic')
    show_gui = DeclareLaunchArgument(
        'show_gui', default_value='false',
        description='Enable package GUI')
    topic_obstacles_gridmap_sub = DeclareLaunchArgument(
        'topic_obstacles_gridmap_sub', default_value='/obstacles_gridmap',
        description='Map subscription topic')
    topic_obstacles_sub = DeclareLaunchArgument(
        'topic_obstacles_sub', default_value='/obstacles_pointcloud',
        description='Obstacles subscription topic')
    topic_localization_sub = DeclareLaunchArgument(
        'topic_localization_sub', default_value='/mrpt_pose',
        description='Localization subscription topic')
    topic_odometry_sub = DeclareLaunchArgument(
        'topic_odometry_sub', default_value='odom',
        description='Odometry subscription topic')
    topic_replan_sub = DeclareLaunchArgument(
        'topic_replan_sub', default_value='/replan',
        description='Replan subscription topic')
    topic_cmd_vel_pub = DeclareLaunchArgument(
        'topic_cmd_vel_pub', default_value='/enq_motion',
        description='Command velocity publish topic')
    topic_wp_seq_pub = DeclareLaunchArgument(
        'topic_wp_seq_pub', default_value='/waypoints',
        description='Waypoints sequence publish topic')

    planner_parameters_arg = DeclareLaunchArgument(
        'planner_parameters', default_value=os.path.join(myDir, 'configs', 'params', 'planner-params.yaml'),
        description='Path to planner-params.yaml configuration file')

    ptg_ini_arg = DeclareLaunchArgument(
        'ptg_ini', default_value=os.path.join(myDir, 'configs', 'ini', 'ptgs_jackal.ini'),
        description='Path to PTG .ini configuration file defining the families of trajectories to use')

    global_costmap_parameters_arg = DeclareLaunchArgument(
        'global_costmap_parameters', default_value=os.path.join(myDir, 'configs', 'params', 'costmap-obstacles.yaml'),
        description='Path to global_costmap_parameters.yaml configuration file')

    prefer_waypoints_parameters_arg = DeclareLaunchArgument(
        'prefer_waypoints_parameters', default_value=os.path.join(myDir, 'configs', 'params', 'costmap-prefer-waypoints.yaml'),
        description='Path to prefer_waypoints_parameters.yaml configuration file')

    # Node configuration
    tps_astar_nav_node = Node(
        package='mrpt_tps_astar_planner',
        executable='mrpt_tps_astar_planner_node',
        name='mrpt_tps_astar_planner_node',
        output='screen',
        parameters=[
            {'topic_goal_sub': LaunchConfiguration('topic_goal_sub')},
            {'show_gui': LaunchConfiguration('show_gui')},
            {'topic_obstacles_gridmap_sub': LaunchConfiguration(
                'topic_obstacles_gridmap_sub')},
            {'topic_localization_sub': LaunchConfiguration(
                'topic_localization_sub')},
            {'topic_odometry_sub': LaunchConfiguration('topic_odometry_sub')},
            {'topic_obstacles_sub': LaunchConfiguration(
                'topic_obstacles_sub')},
            {'topic_replan_sub': LaunchConfiguration('topic_replan_sub')},
            {'topic_cmd_vel_pub': LaunchConfiguration('topic_cmd_vel_pub')},
            {'topic_wp_seq_pub': LaunchConfiguration('topic_wp_seq_pub')},
            # Param files:
            {'planner_parameters': LaunchConfiguration('planner_parameters')},
            {'global_costmap_parameters': LaunchConfiguration(
                'global_costmap_parameters')},
            {'prefer_waypoints_parameters': LaunchConfiguration(
                'prefer_waypoints_parameters')},
            {'ptg_ini': LaunchConfiguration('ptg_ini')},
        ]
    )

    # Launch description
    return LaunchDescription([
        topic_goal_sub,
        show_gui,
        topic_obstacles_gridmap_sub,
        topic_localization_sub,
        topic_odometry_sub,
        topic_obstacles_sub,
        topic_replan_sub,
        topic_cmd_vel_pub,
        topic_wp_seq_pub,
        planner_parameters_arg,
        ptg_ini_arg,
        global_costmap_parameters_arg,
        prefer_waypoints_parameters_arg,
        tps_astar_nav_node
    ])
