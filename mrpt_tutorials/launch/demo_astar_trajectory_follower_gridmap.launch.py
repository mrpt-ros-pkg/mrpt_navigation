# ROS 2 launch file for example in mrpt_tutorials
#
# Same as demo_astar_planner_gridmap.launch.py, but uses the newer
# mrpt_trajectory_follower (mpp::TrajectoryFollower) instead of
# mrpt_reactivenav2d to drive the robot along the A* planned path.
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")
    astarDir = get_package_share_directory("mrpt_tps_astar_planner")

    ptg_ini_file = os.path.join(astarDir, 'configs', 'ini', 'ptgs_jackal.ini')

    mrpt_astar_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            astarDir, 'launch', 'tps_astar_planner.launch.py')]),
        launch_arguments={
            'topic_goal_sub': '/goal_pose',
            'show_gui': 'False',
            'topic_obstacles_gridmap_sub': '/mrpt_map/map_gridmap',
            'topic_static_maps': '/mrpt_map/map_gridmap',
            'topic_wp_seq_pub': '/waypoints',
            'ptg_ini': ptg_ini_file,
        }.items()
    )

    mrpt_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_map_server'), 'launch',
            'mrpt_map_server.launch.py')]),
        launch_arguments={
            'map_yaml_file': os.path.join(tutsDir, 'maps', 'demo_world2.yaml'),
        }.items()
    )

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            {
                "world_file": os.path.join(tutsDir, 'mvsim', 'demo_world2.world.xml'),
                "do_fake_localization": False,
                "headless": True,
            }]
    )

    # Launch for pf_localization:
    pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch',
            'localization.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'log_level_core': 'INFO',
            'topic_sensors_2d_scan': '/laser1',
            'topic_sensors_point_clouds': '',
            'base_link_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
        }.items()
    )

    # Launch for mrpt_pointcloud_pipeline:
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            'use_composable': 'false',
            'log_level': 'INFO',
            'scan_topic_name': '/laser1, /laser2',
            'points_topic_name': '/lidar1_points',
            'filter_output_topic_name': '/local_map_pointcloud',
            'time_window': '0.20',
            'show_gui': 'False',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    # Launch for mrpt_trajectory_follower:
    trajectory_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_trajectory_follower'), 'launch',
            'trajectory_follower.launch.py')]),
        launch_arguments={
            'frame_id_map': 'map',
            'frame_id_robot': 'base_link',
            'topic_path_sub': '/waypoints_path',  # published by mrpt_tps_astar_planner
            'topic_odom_sub': '/odom',
            'topic_cmd_vel_pub': '/cmd_vel',
            'ptg_ini': ptg_ini_file,
            'log_level': 'INFO',
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [os.path.join(tutsDir, 'rviz2', 'gridmap.rviz')]]
    )
    return LaunchDescription([
        mrpt_map_launch,
        mrpt_astar_planner_launch,
        mvsim_node,
        pf_localization_launch,
        pointcloud_pipeline_launch,
        trajectory_follower_launch,
        rviz2_node
    ])
