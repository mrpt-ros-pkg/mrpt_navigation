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
        'topic_obstacles_gridmap_sub', default_value='',
        description='Topic(s) (comma-separated) to subscribe for incoming occupancy grid maps with obstacles')

    topic_obstacles_sub = DeclareLaunchArgument(
        'topic_obstacles_sub', default_value='',
        description='Topic(s) (comma-separated) to subscribe for incoming point clouds with obstacles')

    topic_static_maps = DeclareLaunchArgument(
        'topic_static_maps', default_value='',
        description='Topic(s) (comma-separated) that shall be subscribed using transient local QoS. To be used with static map sources. Topic(s) are to be provided also in the lists "topic_obstacles_gridmap_sub" or "topic_obstacles_sub"')

    topic_wp_seq_pub = DeclareLaunchArgument(
        'topic_wp_seq_pub', default_value='/waypoints',
        description='Waypoints sequence publish topic')

    frame_id_robot = DeclareLaunchArgument(
        'frame_id_robot', default_value='base_link',
        description='frame_id for the robot')

    frame_id_map = DeclareLaunchArgument(
        'frame_id_map', default_value='map',
        description='frame_id for the map')

    mid_waypoints_allowed_distance = DeclareLaunchArgument(
        'mid_waypoints_allowed_distance', default_value='0.5',
        description='allowed_distance field of middle waypoints of the interpolated path')

    final_waypoint_allowed_distance = DeclareLaunchArgument(
        'final_waypoint_allowed_distance', default_value='0.4',
        description='allowed_distance field of final waypoint of the interpolated path')

    mid_waypoints_allow_skip = DeclareLaunchArgument(
        'mid_waypoints_allow_skip', default_value='true',
        description='allow_skip field of middle waypoints of the interpolated path')

    final_waypoint_allow_skip = DeclareLaunchArgument(
        'final_waypoint_allow_skip', default_value='false',
        description='allow_skip field of final waypoint of the interpolated path')

    mid_waypoints_ignore_heading = DeclareLaunchArgument(
        'mid_waypoints_ignore_heading', default_value='false',
        description='ignore_heading field of middle waypoints of the interpolated path')

    final_waypoint_ignore_heading = DeclareLaunchArgument(
        'final_waypoint_ignore_heading', default_value='false',
        description='ignore_heading field of final waypoint of the interpolated path')

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

    problem_world_bbox_margin_arg = DeclareLaunchArgument(
        'problem_world_bbox_margin', default_value='2.0',
        description='The distance [m] to add as a margin all around the computed problem world bounding box')

    problem_world_bbox_ignore_obstacles_arg = DeclareLaunchArgument(
        'problem_world_bbox_ignore_obstacles', default_value='False',
        description='If True, the bounding box of obstacles is ignored while computing the problem world bounding box')

    astar_skip_refine_arg = DeclareLaunchArgument(
        'astar_skip_refine', default_value='False',
        description='If True, the refine stage after A* will be skipped')

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
            {'topic_obstacles_sub': LaunchConfiguration(
                'topic_obstacles_sub')},
            {'topic_static_maps': LaunchConfiguration('topic_static_maps')},
            {'topic_wp_seq_pub': LaunchConfiguration('topic_wp_seq_pub')},
            {'frame_id_robot': LaunchConfiguration('frame_id_robot')},
            {'frame_id_map': LaunchConfiguration('frame_id_map')},
            {'mid_waypoints_allowed_distance': LaunchConfiguration(
                'mid_waypoints_allowed_distance')},
            {'final_waypoint_allowed_distance': LaunchConfiguration(
                'final_waypoint_allowed_distance')},
            {'mid_waypoints_allow_skip': LaunchConfiguration(
                'mid_waypoints_allow_skip')},
            {'final_waypoint_allow_skip': LaunchConfiguration(
                'final_waypoint_allow_skip')},
            {'mid_waypoints_ignore_heading': LaunchConfiguration(
                'mid_waypoints_ignore_heading')},
            {'final_waypoint_ignore_heading': LaunchConfiguration(
                'final_waypoint_ignore_heading')},
            {'problem_world_bbox_margin': LaunchConfiguration(
                'problem_world_bbox_margin')},
            {'problem_world_bbox_ignore_obstacles': LaunchConfiguration(
                'problem_world_bbox_ignore_obstacles')},
            {'astar_skip_refine': LaunchConfiguration('astar_skip_refine')},
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
        topic_obstacles_sub,
        topic_static_maps,
        topic_wp_seq_pub,
        frame_id_robot,
        frame_id_map,
        mid_waypoints_allowed_distance,
        final_waypoint_allowed_distance,
        mid_waypoints_allow_skip,
        final_waypoint_allow_skip,
        mid_waypoints_ignore_heading,
        final_waypoint_ignore_heading,
        planner_parameters_arg,
        ptg_ini_arg,
        global_costmap_parameters_arg,
        prefer_waypoints_parameters_arg,
        problem_world_bbox_margin_arg,
        problem_world_bbox_ignore_obstacles_arg,
        astar_skip_refine_arg,
        tps_astar_nav_node
    ])
