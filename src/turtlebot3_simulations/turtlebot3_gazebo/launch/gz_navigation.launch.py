#!/usr/bin/env python3
#
# Combined launch: Gazebo Harmonic + TurtleBot3 + Nav2 Navigation
#
# Usage:
#   ros2 launch turtlebot3_gazebo gz_navigation.launch.py map:=/path/to/map.yaml
#   ros2 launch turtlebot3_gazebo gz_navigation.launch.py world:=turtlebot3_world map:=/path/to/map.yaml

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')


def generate_launch_description():
    # Package directories
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    house_pkg = get_package_share_directory('aws_robomaker_small_house_world')
    launch_file_dir = os.path.join(gazebo_pkg, 'launch')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # Map file
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(nav2_pkg, 'map', 'map.yaml'))

    # Nav2 params
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        default_param_file = os.path.join(nav2_pkg, 'param', ROS_DISTRO, param_file_name)
    else:
        default_param_file = os.path.join(nav2_pkg, 'param', param_file_name)
    params_file = LaunchConfiguration('params_file', default=default_param_file)

    # World selection: defaults to small_house
    world_files = {
        'small_house': os.path.join(house_pkg, 'worlds', 'small_house.world'),
        'turtlebot3_world': os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_world.world'),
    }
    default_world = world_files.get('small_house')

    # RViz config
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'tb3_navigation2.rviz')

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

    # ── Nav2 bringup ──
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items())

    # ── RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ── Build launch description ──
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'world', default_value='small_house',
        description='World to load: small_house or turtlebot3_world'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_pkg, 'map', 'map.yaml'),
        description='Full path to map yaml file'))
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Full path to Nav2 params file'))

    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_server)
    ld.add_action(gz_gui)
    ld.add_action(robot_state_pub)
    ld.add_action(spawn_tb3)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_node)

    return ld
