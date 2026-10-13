# ROS 2 launch file for example in mrpt_tutorials
#
# A* planner (mrpt_tps_astar_planner) + reactive navigator (mrpt_reactivenav2d)
# on a simulated differential-drive robot.
#
# All nodes except mvsim run on the simulation clock published by mvsim
# (launch argument use_sim_time, default True), so they stay consistent with
# sensor timestamps even if the simulation runs slower than real time.
#
# See repo online: https://github.com/mrpt-ros-pkg/mrpt_navigation
#

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    tutsDir = get_package_share_directory("mrpt_tutorials")
    # print('tutsDir       : ' + tutsDir)

    mrpt_astar_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_tps_astar_planner'), 'launch',
            'tps_astar_planner.launch.py')]),
        launch_arguments={
            'topic_goal_sub': '/goal_pose',
            'show_gui': 'False',
            'topic_obstacles_gridmap_sub': '/mrpt_map/map_gridmap',
            'topic_static_maps': '/mrpt_map/map_gridmap',
            'topic_wp_seq_pub': '/waypoints',
            # 'problem_world_bbox_ignore_obstacles': 'True',
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
            # os.path.join(tutsDir, 'params', 'mvsim_ros2_params.yaml'),
            {
                # Diff-drive robot, matching the PTGs of the planner and the
                # reactive navigator (see the Ackermann car demo in
                # demo_astar_trajectory_follower_gridmap.launch.py):
                "world_file": os.path.join(tutsDir, 'mvsim', 'demo_world2_diffdrive.world.xml'),
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
            # Start localized at the robot pose in the mvsim world file:
            'pf_params_overrides_file': os.path.join(
                tutsDir, 'params', 'pf-initial-pose-demo_world2.yaml'),
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
            'scan_topic_name': '/laser1',
            'points_topic_name': '/lidar1_points',
            'filter_output_topic_name': '/local_map_pointcloud',
            'time_window': '0.20',
            'show_gui': 'False',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    # Launch for mrpt_reactivenav2d:
    node_rnav2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_reactivenav2d'), 'launch',
            'rnav.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            # 'save_nav_log': 'True',
            'frameid_robot': 'base_link',
            'frameid_reference': 'map',
            'topic_reactive_nav_goal': '/rnav_goal',  # we don't publish this topic
            'topic_wp_seq': '/waypoints',            # only this one instead.
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=[
                '-d', [os.path.join(tutsDir, 'rviz2', 'gridmap.rviz')]]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='True',
            description='Whether to launch RViz2 (False for headless runs)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Run the nodes on the simulation clock from mvsim'),
        mvsim_node,
        GroupAction([
            SetParameter(name='use_sim_time',
                         value=LaunchConfiguration('use_sim_time')),
            mrpt_map_launch,
            mrpt_astar_planner_launch,
            pf_localization_launch,
            pointcloud_pipeline_launch,
            node_rnav2d_launch,
            rviz2_node,
        ]),
    ])
