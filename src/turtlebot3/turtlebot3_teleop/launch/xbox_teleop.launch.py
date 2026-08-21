from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path',
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Command velocity topic',
    )

    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic',
        default_value='joy',
        description='Joy topic name',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.08,
            'autorepeat_rate': 50.0,
        }],
    )

    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_xbox_joy',
        name='teleop_xbox_joy',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'joy_topic': LaunchConfiguration('joy_topic'),
            'linear_axis': 1,
            'angular_axis': 3,
            'button_linear_up': 5,
            'button_linear_down': 4,
            'button_angular_up': 3,
            'button_angular_down': 0,
            'button_stop': 6,
            'button_enable': -1,
            'publish_rate': 30.0,
            'axis_deadzone': 0.08,
            'axis_expo': 1.6,
            'linear_accel_limit': 0.8,
            'angular_accel_limit': 1.8,
        }],
    )

    return LaunchDescription([
        joy_dev_arg,
        cmd_vel_topic_arg,
        joy_topic_arg,
        joy_node,
        teleop_node,
    ])
