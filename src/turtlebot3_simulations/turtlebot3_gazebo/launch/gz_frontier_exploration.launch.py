#!/usr/bin/env python3
#
# Frontier exploration: Gazebo Harmonic + Cartographer SLAM + Nav2 (no AMCL) + explore_lite
#
# Cartographer provides /map topic and map→odom TF — no AMCL needed.
# Nav2 navigation_launch.py handles path planning + local obstacle avoidance.
# explore_lite detects frontiers from the Nav2 global costmap and sends NavigateToPose goals.
#
# Prerequisites (build once):
#   cd ~/dl_hackathon/src && git clone https://github.com/robo-friends/m-explore-ros2.git
#   cd ~/dl_hackathon && colcon build --packages-select explore_lite
#
# Usage:
#   ros2 launch turtlebot3_gazebo gz_frontier_exploration.launch.py
#   ros2 launch turtlebot3_gazebo gz_frontier_exploration.launch.py world:=turtlebot3_world

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg       = get_package_share_directory('turtlebot3_gazebo')
    cartographer_pkg = get_package_share_directory('turtlebot3_cartographer')
    ros_gz_sim_pkg   = get_package_share_directory('ros_gz_sim')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    house_pkg        = get_package_share_directory('aws_robomaker_small_house_world')
    launch_dir       = os.path.join(gazebo_pkg, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz     = LaunchConfiguration('use_rviz',     default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='-2.0')
    y_pose       = LaunchConfiguration('y_pose',       default='-0.5')

    world_files = {
        'small_house':     os.path.join(house_pkg,   'worlds', 'small_house.world'),
        'turtlebot3_world': os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_world.world'),
    }

    cartographer_config_dir    = os.path.join(cartographer_pkg, 'config')
    configuration_basename     = 'turtlebot3_lds_2d_exploration.lua'
    nav2_params_file           = os.path.join(gazebo_pkg, 'params', 'nav2_exploration_waffle.yaml')
    explore_params_file        = os.path.join(gazebo_pkg, 'params', 'explore_lite_params.yaml')
    rviz_config                = os.path.join(cartographer_pkg, 'rviz', 'tb3_cartographer.rviz')

    # ── GZ_SIM_RESOURCE_PATH ──
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            os.pathsep,
            os.path.join(gazebo_pkg, 'models'),
            os.pathsep,
            house_pkg,
            os.pathsep,
            os.path.join(house_pkg, 'models'),
            os.pathsep,
            os.path.join(house_pkg, 'worlds'),
        ])

    # ── Gazebo server ──
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world_files['small_house']],
            'on_exit_shutdown': 'true',
        }.items())

    # ── Gazebo GUI ──
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v4 '}.items())

    # ── Robot state publisher ──
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # ── Spawn TB3 + ros_gz bridge ──
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items())

    # ── Cartographer SLAM ──
    # Publishes /map (OccupancyGrid) + map→odom TF — replaces AMCL
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    # ── Nav2 navigation only (no AMCL, no map_server) ──
    # navigation_launch.py starts: controller, planner, behaviors, bt_navigator, lifecycle_manager
    # Cartographer already provides the map TF — nav2 just does path planning + control
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
        }.items())

    # ── explore_lite frontier exploration ──
    # Uses raw /map from Cartographer (not Nav2 costmap) — works immediately on startup.
    # Config mirrors the reference VLM-nav repo params exactly.
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    # ── VLM semantic detection (runs in background during exploration) ──
    # Detects rooms/objects from camera, stores locations in ~/dl_hackathon/semantic_map.json
    vlm_node = Node(
        package='vlm_nav',
        executable='vlm_detection',
        name='vlm_detection',
        output='screen')

    # ── RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz',     default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose',       default_value='-2.0'))
    ld.add_action(DeclareLaunchArgument('y_pose',       default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='small_house',
        description='World to load: small_house or turtlebot3_world'))

    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_server)
    ld.add_action(gz_gui)
    ld.add_action(robot_state_pub)
    ld.add_action(spawn_tb3)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(nav2_navigation)
    # Delay explore_lite: Nav2 global costmap needs ~15s to activate and populate
    # free cells around robot before explore_lite BFS can find a starting cell
    ld.add_action(TimerAction(period=25.0, actions=[explore_node]))
    ld.add_action(vlm_node)

    if use_rviz:
        ld.add_action(rviz_node)

    return ld
