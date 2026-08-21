#!/usr/bin/env python3
"""
Dynamic Obstacle Mover for Gz Harmonic
Replaces the Gazebo Classic C++ obstacle plugins (obstacle1.cc - obstacle6.cc)
by using the ros_gz SetEntityPose service to animate obstacle positions.

Each obstacle follows a looping keyframed patrol route extracted from the
original turtlebot3_drlnav obstacle plugins.

Usage:
  ros2 run turtlebot3_drl obstacle_mover
"""

import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


# ===================================================================== #
# Obstacle patrol routes — extracted from original C++ PoseAnimation     #
# Each entry: (time_s, x_offset, y_offset) relative to spawn position   #
# ===================================================================== #

OBSTACLE_ROUTES = {
    1: {
        'period': 160.0,
        'keyframes': [
            (0,   0.0,  0.0),
            (10, -0.5, -1.0),
            (50, -3.5, -1.0),
            (70, -3.7, -3.0),
            (90, -3.5, -1.0),
            (130,-0.5, -1.0),
            (140, 0.0,  0.0),
            (160, 0.0,  0.0),
        ]
    },
    2: {
        'period': 130.0,
        'keyframes': [
            (0,   0.0,  0.0),
            (10,  0.7,  0.2),
            (40,  2.5,  3.5),
            (55,  0.3,  3.5),
            (85,  3.5,  1.8),
            (100, 3.5,  0.0),
            (110, 2.0,  0.5),
            (115, 1.5,  1.0),
            (120, 1.0,  0.5),
            (125, 0.5,  0.1),
            (130, 0.0,  0.0),
        ]
    },
    3: {
        'period': 165.0,
        'keyframes': [
            (0,   0.0,  0.0),
            (10, -1.0,  0.2),
            (40, -2.0,  1.0),
            (55, -3.5,  0.0),
            (85, -2.5,  1.5),
            (110, 0.0,  0.0),
            (130,-1.0,  2.0),
            (145,-2.0,  1.0),
            (165, 0.0,  0.0),
        ]
    },
    4: {
        'period': 170.0,
        'keyframes': [
            (0,   0.0,  0.0),
            (10,  0.0, -3.2),
            (30,  2.0, -2.7),
            (40,  0.0,  0.0),
            (60,  0.0, -3.2),
            (80,  2.0, -2.7),
            (110, 0.0,  0.0),
            (130, 0.0, -3.2),
            (150, 2.0, -2.7),
            (170, 0.0,  0.0),
        ]
    },
    5: {
        'period': 200.0,
        'keyframes': [
            (0,   0.0,  0.0),
            (10,  0.7,  1.0),
            (40,  2.5,  2.0),
            (55,  0.0,  2.0),
            (85,  0.0, -1.0),
            (110, 2.0,  0.0),
            (125, 4.0,  0.0),
            (145, 3.0, -2.0),
            (170, 2.0,  0.0),
            (185, 2.0,  2.0),
            (200, 0.0,  0.0),
        ]
    },
    6: {
        'period': 170.0,
        'keyframes': [
            (0,   0.0,  0.0),
            (10, -1.0,  0.0),
            (40, -1.0,  2.0),
            (55, -1.5,  0.0),
            (85,  0.0,  2.0),
            (120,-4.0, -1.8),
            (130,-3.0, -1.8),
            (145,-2.5,  1.0),
            (170, 0.0,  0.0),
        ]
    },
}

# Stage configurations: which obstacles are active and their spawn positions
STAGE_OBSTACLES = {
    1: [],  # Empty arena
    2: [{'id': 1, 'spawn': (0.0, 0.0), 'moving': False}],  # 1 static
    3: [{'id': 1, 'spawn': (0.5, 0.5), 'moving': True},
        {'id': 2, 'spawn': (-0.5, -0.5), 'moving': True}],
    4: [{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True}],
    5: [{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True},
        {'id': 3, 'spawn': (2.0, -2.0), 'moving': True},
        {'id': 4, 'spawn': (-2.0, 2.0), 'moving': True}],
    6: [{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True},
        {'id': 3, 'spawn': (2.0, -2.0), 'moving': True},
        {'id': 4, 'spawn': (-2.0, 2.0), 'moving': True}],
    7: [{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True}],
    8: [{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True}],
    9: [{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True},
        {'id': 3, 'spawn': (1.5, -1.0), 'moving': True},
        {'id': 4, 'spawn': (-1.0, 1.5), 'moving': True},
        {'id': 5, 'spawn': (0.5, -1.8), 'moving': True},
        {'id': 6, 'spawn': (-1.5, -0.5), 'moving': True}],
    10:[{'id': 1, 'spawn': (2.0, 2.0), 'moving': True},
        {'id': 2, 'spawn': (-2.0, -2.0), 'moving': True},
        {'id': 3, 'spawn': (1.5, -1.0), 'moving': True},
        {'id': 4, 'spawn': (-1.0, 1.5), 'moving': True},
        {'id': 5, 'spawn': (0.5, -1.8), 'moving': True},
        {'id': 6, 'spawn': (-1.5, -0.5), 'moving': True}],
}


def lerp(a, b, t):
    """Linear interpolation between a and b by factor t (0-1)"""
    return a + (b - a) * t


def get_position_at_time(route, t):
    """Interpolate position from keyframes at time t"""
    period = route['period']
    t = t % period  # Loop
    keyframes = route['keyframes']

    # Find surrounding keyframes
    for i in range(len(keyframes) - 1):
        t0, x0, y0 = keyframes[i]
        t1, x1, y1 = keyframes[i + 1]
        if t0 <= t <= t1:
            frac = (t - t0) / (t1 - t0) if t1 != t0 else 0
            return lerp(x0, x1, frac), lerp(y0, y1, frac)

    # Fallback: last keyframe
    return keyframes[-1][1], keyframes[-1][2]


class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')

        # Read current stage
        try:
            with open('/tmp/drlnav_current_stage.txt', 'r') as f:
                self.stage = int(f.read().strip())
        except FileNotFoundError:
            self.stage = 4
            self.get_logger().warn('Stage file not found, defaulting to stage 4')

        self.obstacles = STAGE_OBSTACLES.get(self.stage, [])
        self.get_logger().info(f'Obstacle mover: stage {self.stage}, '
                               f'{len(self.obstacles)} obstacles '
                               f'({sum(1 for o in self.obstacles if o["moving"])} moving)')

        if not self.obstacles:
            self.get_logger().info('No obstacles for this stage, node idle.')
            return

        # Publish obstacle odom for the DRL environment (single topic, child_frame_id identifies obstacle)
        self.obstacle_odom_pub = self.create_publisher(Odometry, 'obstacle/odom', 10)

        self.start_time = time.time()

        # Update at 10Hz
        self.timer = self.create_timer(0.1, self.update_obstacles)

    def update_obstacles(self):
        elapsed = time.time() - self.start_time

        for obs in self.obstacles:
            obs_id = obs['id']
            spawn_x, spawn_y = obs['spawn']

            if obs['moving'] and obs_id in OBSTACLE_ROUTES:
                route = OBSTACLE_ROUTES[obs_id]
                dx, dy = get_position_at_time(route, elapsed)
            else:
                dx, dy = 0.0, 0.0

            world_x = spawn_x + dx
            world_y = spawn_y + dy

            # Set pose in Gz Harmonic
            self._set_obstacle_pose(f'drl_obstacle{obs_id}', world_x, world_y)

            # Publish odom for DRL environment
            self._publish_odom(obs_id, world_x, world_y)

    def _set_obstacle_pose(self, name, x, y):
        """Move obstacle via gz service CLI (Gz Transport, not ROS)"""
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/default/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '100',
                 '--req', f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.0}}'],
                capture_output=True, timeout=1
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

    def _publish_odom(self, obs_id, x, y):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = f'obstacle{obs_id}'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        self.obstacle_odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
