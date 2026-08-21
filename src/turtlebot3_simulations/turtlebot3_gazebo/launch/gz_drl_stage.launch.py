#!/usr/bin/env python3
#
# DRL Training Launch File for Gz Harmonic (Multi-Stage)
# Launches: Gz Harmonic sim + TurtleBot3 spawn + ros_gz bridge
#
# Usage:
#   ros2 launch turtlebot3_gazebo gz_drl_stage.launch.py              # stage 4 default
#   ros2 launch turtlebot3_gazebo gz_drl_stage.launch.py stage:=1     # empty arena
#   ros2 launch turtlebot3_gazebo gz_drl_stage.launch.py stage:=9     # hard mode

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    stage = LaunchConfiguration('stage').perform(context)

    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    desc_pkg = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(gazebo_pkg, 'worlds', f'turtlebot3_drl_stage{stage}.world')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'turtlebot3_waffle.urdf')
    bridge_params = os.path.join(gazebo_pkg, 'params', 'turtlebot3_waffle_bridge.yaml')
    model_sdf = os.path.join(gazebo_pkg, 'models', 'turtlebot3_waffle', 'model.sdf')

    if not os.path.exists(world_file):
        raise FileNotFoundError(f"Stage {stage} world not found: {world_file}\n"
                                f"Available stages: 1-10")

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Write stage for DRL nodes
    with open('/tmp/drlnav_current_stage.txt', 'w') as f:
        f.write(f"{stage}\n")

    return [
        # Gz Harmonic server + GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # Spawn TurtleBot3
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'turtlebot3_waffle',
                '-file', model_sdf,
                '-x', '-0.7', '-y', '0.0', '-z', '0.01',
            ],
            output='screen',
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
            }],
            output='screen',
        ),

        # ros_gz bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_params}],
            output='screen',
        ),

        # ros_gz image bridge
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
        ),

        # Dynamic obstacle mover (animates obstacles based on stage)
        Node(
            package='turtlebot3_drl',
            executable='obstacle_mover',
            output='screen',
        ),
    ]


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        # GZ_SIM_RESOURCE_PATH
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_pkg, 'models')),

        DeclareLaunchArgument('stage', default_value='4',
                              description='DRL training stage (1=empty, 4=medium, 9=hard)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        OpaqueFunction(function=launch_setup),
    ])
