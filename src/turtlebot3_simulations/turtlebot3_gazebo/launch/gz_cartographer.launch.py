#!/usr/bin/env python3
#
# Combined launch: Gazebo Harmonic + TurtleBot3 + Cartographer SLAM
#
# Usage:
#   ros2 launch turtlebot3_gazebo gz_cartographer.launch.py
#   ros2 launch turtlebot3_gazebo gz_cartographer.launch.py world:=turtlebot3_world

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    cartographer_pkg = get_package_share_directory('turtlebot3_cartographer')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    house_pkg = get_package_share_directory('aws_robomaker_small_house_world')
    launch_file_dir = os.path.join(gazebo_pkg, 'launch')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # World selection: 'small_house' (default) or 'turtlebot3_world'
    world_name = LaunchConfiguration('world', default='small_house')

    # Map world name → file path
    world_files = {
        'small_house': os.path.join(house_pkg, 'worlds', 'small_house.world'),
        'turtlebot3_world': os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_world.world'),
    }

    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(cartographer_pkg, 'config'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='turtlebot3_lds_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # RViz config
    rviz_config = os.path.join(cartographer_pkg, 'rviz', 'tb3_cartographer.rviz')

    # ── Environment: Gz resource paths ──
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

    # ── Gazebo Harmonic server ──
    # We use the small_house world by default; override with world:=turtlebot3_world
    default_world = world_files.get('small_house')
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', default_world],
            'on_exit_shutdown': 'true',
        }.items())

    # ── Gazebo Harmonic GUI ──
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v4 '}.items())

    # ── Robot state publisher ──
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # ── Spawn TB3 + ros_gz bridge ──
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items())

    # ── Cartographer SLAM ──
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename])

    # ── Occupancy grid from Cartographer ──
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec])

    # ── RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen')

    # ── Build launch description ──
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'world', default_value='small_house',
        description='World to load: small_house or turtlebot3_world'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument('resolution', default_value='0.05'))
    ld.add_action(DeclareLaunchArgument('publish_period_sec', default_value='1.0'))

    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_server)
    ld.add_action(gz_gui)
    ld.add_action(robot_state_pub)
    ld.add_action(spawn_tb3)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
