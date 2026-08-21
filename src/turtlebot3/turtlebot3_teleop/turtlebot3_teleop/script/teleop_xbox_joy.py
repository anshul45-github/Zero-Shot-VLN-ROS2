#!/usr/bin/env python3

import os

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy
from sensor_msgs.msg import Joy


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 1.5
WAFFLE_MAX_ANG_VEL = 1.82


class XboxJoyTeleop(Node):
    def __init__(self):
        super().__init__('teleop_xbox_joy')

        turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
        if turtlebot3_model == 'burger':
            self.max_linear_limit = BURGER_MAX_LIN_VEL
            self.max_angular_limit = BURGER_MAX_ANG_VEL
            default_linear_limit = 0.22
            default_angular_limit = 1.0
        else:
            self.max_linear_limit = WAFFLE_MAX_LIN_VEL
            self.max_angular_limit = WAFFLE_MAX_ANG_VEL
            default_linear_limit = 0.26
            default_angular_limit = 1.2

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('joy_timeout', 0.5)

        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('axis_deadzone', 0.1)
        self.declare_parameter('axis_expo', 1.6)
        self.declare_parameter('linear_axis_scale', 1.0)
        self.declare_parameter('angular_axis_scale', 1.0)

        self.declare_parameter('button_linear_up', 5)      # RB
        self.declare_parameter('button_linear_down', 4)    # LB
        self.declare_parameter('button_angular_up', 3)     # Y
        self.declare_parameter('button_angular_down', 0)   # A
        self.declare_parameter('button_stop', 6)           # BACK
        self.declare_parameter('button_enable', -1)        # Disabled by default

        self.declare_parameter('linear_speed_step', 0.1)
        self.declare_parameter('angular_speed_step', 0.1)
        self.declare_parameter('initial_linear_limit', default_linear_limit)
        self.declare_parameter('initial_angular_limit', default_angular_limit)

        self.declare_parameter('linear_accel_limit', 0.8)   # m/s^2
        self.declare_parameter('angular_accel_limit', 1.8)  # rad/s^2

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        joy_topic = self.get_parameter('joy_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.axis_deadzone = float(self.get_parameter('axis_deadzone').value)
        self.axis_expo = max(1.0, float(self.get_parameter('axis_expo').value))
        self.linear_axis_scale = float(self.get_parameter('linear_axis_scale').value)
        self.angular_axis_scale = float(self.get_parameter('angular_axis_scale').value)

        self.button_linear_up = int(self.get_parameter('button_linear_up').value)
        self.button_linear_down = int(self.get_parameter('button_linear_down').value)
        self.button_angular_up = int(self.get_parameter('button_angular_up').value)
        self.button_angular_down = int(self.get_parameter('button_angular_down').value)
        self.button_stop = int(self.get_parameter('button_stop').value)
        self.button_enable = int(self.get_parameter('button_enable').value)

        self.linear_speed_step = float(self.get_parameter('linear_speed_step').value)
        self.angular_speed_step = float(self.get_parameter('angular_speed_step').value)

        initial_linear_limit = float(self.get_parameter('initial_linear_limit').value)
        initial_angular_limit = float(self.get_parameter('initial_angular_limit').value)
        self.current_linear_limit = self._clamp(initial_linear_limit, 0.0, self.max_linear_limit)
        self.current_angular_limit = self._clamp(initial_angular_limit, 0.0, self.max_angular_limit)

        self.linear_accel_limit = max(0.01, float(self.get_parameter('linear_accel_limit').value))
        self.angular_accel_limit = max(0.01, float(self.get_parameter('angular_accel_limit').value))

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

        self.last_joy_time = self.get_clock().now()
        self.previous_buttons = []

        qos = QoSProfile(depth=10)
        self.ros_distro = os.environ.get('ROS_DISTRO', '')
        if self.ros_distro == 'humble':
            self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        else:
            self.publisher = self.create_publisher(TwistStamped, self.cmd_vel_topic, qos)

        self.subscription = self.create_subscription(Joy, joy_topic, self._joy_callback, qos)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_loop)

        self.get_logger().info(
            'Xbox teleop ready. '
            f'model={turtlebot3_model} '
            f'linear_limit={self.current_linear_limit:.2f}/{self.max_linear_limit:.2f} '
            f'angular_limit={self.current_angular_limit:.2f}/{self.max_angular_limit:.2f}'
        )

    @staticmethod
    def _clamp(value, low, high):
        if value < low:
            return low
        if value > high:
            return high
        return value

    @staticmethod
    def _apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def _shape_axis(value, expo):
        # expo > 1.0 gives finer low-speed control around center stick.
        return (abs(value) ** expo) * (1.0 if value >= 0.0 else -1.0)

    @staticmethod
    def _make_simple_profile(output_vel, input_vel, step):
        if input_vel > output_vel:
            return min(input_vel, output_vel + step)
        if input_vel < output_vel:
            return max(input_vel, output_vel - step)
        return input_vel

    @staticmethod
    def _is_pressed(buttons, index):
        return index >= 0 and index < len(buttons) and buttons[index] == 1

    def _button_rising_edge(self, buttons, index):
        now_pressed = self._is_pressed(buttons, index)
        was_pressed = index >= 0 and index < len(self.previous_buttons) and self.previous_buttons[index] == 1
        return now_pressed and not was_pressed

    def _axis_value(self, axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        return float(axes[index])

    def _joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        buttons = msg.buttons

        if self._button_rising_edge(buttons, self.button_linear_up):
            self.current_linear_limit = self._clamp(
                self.current_linear_limit + self.linear_speed_step,
                0.0,
                self.max_linear_limit,
            )
            self.get_logger().info(f'Linear speed limit: {self.current_linear_limit:.2f} m/s')

        if self._button_rising_edge(buttons, self.button_linear_down):
            self.current_linear_limit = self._clamp(
                self.current_linear_limit - self.linear_speed_step,
                0.0,
                self.max_linear_limit,
            )
            self.get_logger().info(f'Linear speed limit: {self.current_linear_limit:.2f} m/s')

        if self._button_rising_edge(buttons, self.button_angular_up):
            self.current_angular_limit = self._clamp(
                self.current_angular_limit + self.angular_speed_step,
                0.0,
                self.max_angular_limit,
            )
            self.get_logger().info(f'Angular speed limit: {self.current_angular_limit:.2f} rad/s')

        if self._button_rising_edge(buttons, self.button_angular_down):
            self.current_angular_limit = self._clamp(
                self.current_angular_limit - self.angular_speed_step,
                0.0,
                self.max_angular_limit,
            )
            self.get_logger().info(f'Angular speed limit: {self.current_angular_limit:.2f} rad/s')

        if self._is_pressed(buttons, self.button_stop):
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.previous_buttons = list(buttons)
            return

        enable_pressed = True
        if self.button_enable >= 0:
            enable_pressed = self._is_pressed(buttons, self.button_enable)

        if not enable_pressed:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.previous_buttons = list(buttons)
            return

        linear_axis_value = self._axis_value(msg.axes, self.linear_axis)
        angular_axis_value = self._axis_value(msg.axes, self.angular_axis)

        linear_axis_value = self._apply_deadzone(linear_axis_value, self.axis_deadzone)
        angular_axis_value = self._apply_deadzone(angular_axis_value, self.axis_deadzone)

        linear_axis_value = self._shape_axis(linear_axis_value, self.axis_expo)
        angular_axis_value = self._shape_axis(angular_axis_value, self.axis_expo)

        self.target_linear_velocity = (
            linear_axis_value * self.linear_axis_scale * self.current_linear_limit
        )
        self.target_angular_velocity = (
            angular_axis_value * self.angular_axis_scale * self.current_angular_limit
        )

        self.previous_buttons = list(buttons)

    def _publish_loop(self):
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if elapsed > self.joy_timeout:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

        linear_step = self.linear_accel_limit / self.publish_rate
        angular_step = self.angular_accel_limit / self.publish_rate

        self.control_linear_velocity = self._make_simple_profile(
            self.control_linear_velocity,
            self.target_linear_velocity,
            linear_step,
        )
        self.control_angular_velocity = self._make_simple_profile(
            self.control_angular_velocity,
            self.target_angular_velocity,
            angular_step,
        )

        if self.ros_distro == 'humble':
            twist = Twist()
            twist.linear.x = self.control_linear_velocity
            twist.angular.z = self.control_angular_velocity
            self.publisher.publish(twist)
        else:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = Clock().now().to_msg()
            twist_stamped.twist.linear.x = self.control_linear_velocity
            twist_stamped.twist.angular.z = self.control_angular_velocity
            self.publisher.publish(twist_stamped)


def main():
    rclpy.init()
    node = XboxJoyTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ros_distro == 'humble':
            stop_msg = Twist()
            node.publisher.publish(stop_msg)
        else:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = Clock().now().to_msg()
            node.publisher.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
