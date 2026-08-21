#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
# Ported to Gz Harmonic by DL Hackathon
#
# Uses gz service CLI for Gazebo operations since ros_gz_bridge
# only bridges topics, not services.

import os
import random
import math
import numpy
import time
import subprocess

from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import EntityFactory

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node

from turtlebot3_msgs.srv import RingGoal
import xml.etree.ElementTree as ET
from ..drl_environment.drl_environment import ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS
from ..common.settings import ENABLE_TRUE_RANDOM_GOALS

NO_GOAL_SPAWN_MARGIN = 0.3  # meters away from any wall


def gz_service(service, req_type, rep_type='gz.msgs.Boolean', req_data='', timeout=3000):
    """Call a Gz Transport service via CLI"""
    try:
        cmd = ['gz', 'service', '-s', service,
               '--reqtype', req_type, '--reptype', rep_type,
               '--timeout', str(timeout)]
        if req_data:
            cmd += ['--req', req_data]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return False


class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        # Goal SDF
        from ament_index_python.packages import get_package_share_directory
        gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
        self.entity_path = os.path.join(
            gazebo_pkg, 'models', 'turtlebot3_drl_world', 'goal_box', 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goal'

        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}, dynamic goals enabled: {ENABLE_DYNAMIC_GOALS}")

        self.prev_x, self.prev_y = -1, -1
        self.goal_x, self.goal_y = 0.5, 0.0

        # Publishers
        self.goal_pose_pub = self.create_publisher(
            Pose, 'goal_pose',
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # Spawn entities via EntityFactory topic (this IS bridged)
        self.spawn_entity_pub = self.create_publisher(
            EntityFactory, '/world/default/create', QoSProfile(depth=10))

        # Services (DRL internal, not Gazebo)
        self.task_succeed_server = self.create_service(RingGoal, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server = self.create_service(RingGoal, 'task_fail', self.task_fail_callback)

        self.obstacle_coordinates = self.get_obstacle_coordinates()
        self.init_callback()

    def init_callback(self):
        self.delete_entity()
        self.reset_simulation()
        self.publish_callback()
        print("Init, goal pose:", self.goal_x, self.goal_y)
        time.sleep(1)

    def publish_callback(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        self.goal_pose_pub.publish(goal_pose)
        self.spawn_entity()

    def task_succeed_callback(self, request, response):
        self.delete_entity()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"success: generate (random) a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def task_fail_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"fail: reset the environment, (random) goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def goal_is_valid(self, goal_x, goal_y):
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0]:
                if goal_y < obstacle[0][1] and goal_y > obstacle[2][1]:
                    return False
        return True

    def generate_random_goal(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0
        while (((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 4) or (not self.goal_is_valid(self.goal_x, self.goal_y))):
            self.goal_x = random.randrange(-25, 25) / 10.0
            self.goal_y = random.randrange(-25, 25) / 10.0
            tries += 1
            if tries > 200:
                print("ERROR: cannot find valid new goal, resetting!")
                self.delete_entity()
                self.reset_simulation()
                self.generate_goal_pose()
                break
        self.publish_callback()

    def generate_dynamic_goal_pose(self, robot_pose_x, robot_pose_y, radius):
        tries = 0
        while(True):
            ring_position = random.uniform(0, 1)
            origin = radius + numpy.random.normal(0, 0.1)
            goal_offset_x = math.cos(2 * math.pi * ring_position) * origin
            goal_offset_y = math.sin(2 * math.pi * ring_position) * origin
            goal_x = robot_pose_x + goal_offset_x
            goal_y = robot_pose_y + goal_offset_y
            if self.goal_is_valid(goal_x, goal_y):
                self.goal_x = goal_x
                self.goal_y = goal_y
                break
            if tries > 100:
                print("Error! couldn't find valid goal position, resetting..")
                self.delete_entity()
                self.reset_simulation()
                self.generate_goal_pose()
                return
            tries += 1
        self.publish_callback()

    def generate_goal_pose(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0

        while ((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 2):
            if self.stage == 11:
                goal_pose_list = [[0.0, 0.0], [0.0, 6.5], [5.0, 5.5], [-2.5, -6.0], [3.0, -4.0], [6.0, -1.0]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            elif self.stage == 8 or self.stage == 9 or self.stage == 12:
                goal_pose_list = [[2.0, 2.0], [2.0, 1.5], [2.0, -0.5], [2.0, -1.0], [2.0, -2.0], [1.3, 1.0],
                                    [1.0, 0.3], [1.0, -2.0], [0.3, -1.0],  [0.0, 2.0], [0.0, -1.0], [-1.0, 1.0],
                                        [-1.0, -1.2], [-2.0, 1.0], [-2.2, 0.0], [-2.0, -2.2], [-2.4, 2.4]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            elif self.stage not in [4, 5, 7]:
                self.goal_x = random.randrange(-15, 16) / 10.0
                self.goal_y = random.randrange(-15, 16) / 10.0
            else:
                goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                                  [-1.9, 1.9], [-1.9,  0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0],
                                  [1.5, -1.0], [-0.5, 1.0], [-1.0, -2.0], [1.8, -0.2], [1.0, -1.9]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            tries += 1
            if tries > 100:
                print("ERROR: distance between goals is small!")
                break
        self.publish_callback()

    def reset_simulation(self):
        """Reset robot pose via gz service CLI"""
        gz_service('/world/default/set_pose',
                   'gz.msgs.Pose',
                   'gz.msgs.Boolean',
                   'name: "turtlebot3_waffle", position: {x: -0.7, y: 0.0, z: 0.01}')

    def delete_entity(self):
        """Delete the goal entity via gz service CLI"""
        gz_service('/world/default/remove',
                   'gz.msgs.Entity',
                   'gz.msgs.Boolean',
                   f'name: "{self.entity_name}", type: MODEL')

    def spawn_entity(self):
        """Spawn goal via EntityFactory publisher (this topic IS bridged by ros_gz)"""
        msg = EntityFactory()
        msg.name = self.entity_name
        msg.sdf = self.entity
        msg.pose = Pose()
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0
        msg.allow_renaming = False
        self.spawn_entity_pub.publish(msg)

    def get_obstacle_coordinates(self):
        from ament_index_python.packages import get_package_share_directory
        gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
        inner_walls_sdf = os.path.join(
            gazebo_pkg, 'models', 'turtlebot3_drl_world', 'inner_walls', 'model.sdf')

        if not os.path.exists(inner_walls_sdf):
            base_path = os.getenv('DRLNAV_BASE_PATH', os.path.expanduser('~/dl_hackathon'))
            inner_walls_sdf = os.path.join(
                base_path, 'src', 'turtlebot3_simulations', 'turtlebot3_gazebo',
                'models', 'turtlebot3_drl_world', 'inner_walls', 'model.sdf')

        tree = ET.parse(inner_walls_sdf)
        root = tree.getroot()
        obstacle_coordinates = []
        for wall in root.find('model').findall('link'):
            pose = wall.find('pose').text.split(" ")
            size = wall.find('collision').find('geometry').find('box').find('size').text.split()
            rotation = float(pose[-1])
            pose_x = float(pose[0])
            pose_y = float(pose[1])
            if rotation == 0:
                size_x = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
            else:
                size_x = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
            point_1 = [pose_x + size_x / 2, pose_y + size_y / 2]
            point_2 = [point_1[0], point_1[1] - size_y]
            point_3 = [point_1[0] - size_x, point_1[1] - size_y]
            point_4 = [point_1[0] - size_x, point_1[1]]
            wall_points = [point_1, point_2, point_3, point_4]
            obstacle_coordinates.append(wall_points)
        return obstacle_coordinates


def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)
    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
