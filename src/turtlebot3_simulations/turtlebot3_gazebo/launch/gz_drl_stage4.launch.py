#!/usr/bin/env python3
#
# DRL Training Launch File for Gz Harmonic
# Launches: Gz Harmonic sim + TurtleBot3 spawn + ros_gz bridge
# Usage: ros2 launch turtlebot3_gazebo gz_drl_stage4.launch.py
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    desc_pkg = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_drl_stage4.world')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'turtlebot3_waffle.urdf')
    bridge_params = os.path.join(gazebo_pkg, 'params', 'turtlebot3_waffle_bridge.yaml')
    model_sdf = os.path.join(gazebo_pkg, 'models', 'turtlebot3_waffle', 'model.sdf')

    # Read URDF for robot_state_publisher
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Write stage number for DRL nodes
    with open('/tmp/drlnav_current_stage.txt', 'w') as f:
        f.write("4\n")

    # GZ_SIM_RESOURCE_PATH: let Gazebo find our models
    gz_resource_path = os.path.join(gazebo_pkg, 'models')
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path
    )

    # Launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Gz Harmonic server + GUI
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn TurtleBot3
    spawn_tb3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-file', model_sdf,
            '-x', '-0.7', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
        }],
        output='screen',
    )

    # ros_gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen',
    )

    # ros_gz image bridge for camera
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_env,
        gui_arg,
        use_sim_time,
        gz_sim,
        spawn_tb3,
        robot_state_pub,
        bridge,
        image_bridge,
    ])
