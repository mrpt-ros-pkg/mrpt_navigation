from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    nav_goal = DeclareLaunchArgument(
        'nav_goal', default_value='[0.0, 0.0, 0.0]',
        description='Navigation goal position')
    start_pose = DeclareLaunchArgument(
        'start_pose', default_value='[0.0, 0.0, 0.0]',
        description='Starting pose')
    start_vel = DeclareLaunchArgument(
        'start_vel', default_value='[0.0, 0.0, 0.0]',
        description='Starting velocity')
    mrpt_gui = DeclareLaunchArgument(
        'mrpt_gui', default_value='false',
        description='Enable MRPT GUI')
    topic_map_sub = DeclareLaunchArgument(
        'topic_map_sub', default_value='map',
        description='Map subscription topic')
    topic_localization_sub = DeclareLaunchArgument(
        'topic_localization_sub', default_value='/mrpt_pose',
        description='Localization subscription topic')
    topic_odometry_sub = DeclareLaunchArgument(
        'topic_odometry_sub', default_value='odom',
        description='Odometry subscription topic')
    topic_obstacles_sub = DeclareLaunchArgument(
        'topic_obstacles_sub', default_value='/map_pointcloud',
        description='Obstacles subscription topic')
    topic_replan_sub = DeclareLaunchArgument(
        'topic_replan_sub', default_value='/replan',
        description='Replan subscription topic')
    topic_cmd_vel_pub = DeclareLaunchArgument(
        'topic_cmd_vel_pub', default_value='/enq_motion',
        description='Command velocity publish topic')
    topic_wp_seq_pub = DeclareLaunchArgument(
        'topic_wp_seq_pub', default_value='/waypoints',
        description='Waypoints sequence publish topic')

    # Node configuration
    tps_astar_nav_node = Node(
        package='mrpt_tps_astar_planner',
        executable='mrpt_tps_astar_planner_node',
        name='mrpt_tps_astar_planner_node',
        output='screen',
        parameters=[
            get_package_share_directory('mrpt_tps_astar_planner') + '/configs/params/planner-params.yaml',
            get_package_share_directory('mrpt_tps_astar_planner') + '/configs/params/costmap-obstacles.yaml',
            get_package_share_directory('mrpt_tps_astar_planner') + '/configs/params/nav-engine-params.yaml',
            get_package_share_directory('mrpt_tps_astar_planner') + '/configs/params/costmap-obstacles.yaml',
            get_package_share_directory('mrpt_tps_astar_planner') + '/configs/params/costmap-prefer-waypoints.yaml',
            get_package_share_directory('mrpt_tps_astar_planner') + '/configs/ini/ptgs_jackal.ini',
            {'nav_goal': LaunchConfiguration('nav_goal')},
            {'start_pose': LaunchConfiguration('start_pose')},
            {'start_vel': LaunchConfiguration('start_vel')},
            {'mrpt_gui': LaunchConfiguration('mrpt_gui')},
            {'topic_map_sub': LaunchConfiguration('topic_map_sub')},
            {'topic_localization_sub': LaunchConfiguration('topic_localization_sub')},
            {'topic_odometry_sub': LaunchConfiguration('topic_odometry_sub')},
            {'topic_obstacles_sub': LaunchConfiguration('topic_obstacles_sub')},
            {'topic_replan_sub': LaunchConfiguration('topic_replan_sub')},
            {'topic_cmd_vel_pub': LaunchConfiguration('topic_cmd_vel_pub')},
            {'topic_wp_seq_pub': LaunchConfiguration('topic_wp_seq_pub')}
        ]
    )

    # Launch description
    return LaunchDescription([
        nav_goal,
        start_pose,
        start_vel,
        mrpt_gui,
        topic_map_sub,
        topic_localization_sub,
        topic_odometry_sub,
        topic_obstacles_sub,
        topic_replan_sub,
        topic_cmd_vel_pub,
        topic_wp_seq_pub,
        tps_astar_nav_node
    ])
