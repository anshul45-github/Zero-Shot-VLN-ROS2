#!/usr/bin/env python3
"""
Semantic Navigation Node.

Usage (after exploration is complete and map saved):
    ros2 run vlm_nav semantic_nav
    >>> Where to go? kitchen
    >>> Where to go? washroom

Loads semantic_map.json, uses CLIP text embeddings to find the best matching
location, and sends a NavigateToPose goal to Nav2.
"""

import json
import os
import sys
from pathlib import Path

import clip
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

SEMANTIC_MAP_PATH = str(Path.home() / 'dl_hackathon' / 'semantic_map.json')


class SemanticNavNode(Node):
    def __init__(self):
        super().__init__('semantic_nav')

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server ...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready.')

        # Load CLIP
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model, _ = clip.load('ViT-B/32', device=self.device)
        self.model.eval()

        # Load semantic map
        if not os.path.exists(SEMANTIC_MAP_PATH):
            self.get_logger().error(f'Semantic map not found: {SEMANTIC_MAP_PATH}')
            self.get_logger().error('Run exploration first to build the map.')
            raise FileNotFoundError(SEMANTIC_MAP_PATH)

        with open(SEMANTIC_MAP_PATH) as f:
            self._map: dict = json.load(f)

        if not self._map:
            raise RuntimeError('Semantic map is empty — run exploration first.')

        # Pre-encode all stored label names
        labels = list(self._map.keys())
        with torch.no_grad():
            tokens = clip.tokenize(labels).to(self.device)
            self._label_feats = self.model.encode_text(tokens)
            self._label_feats = self._label_feats / self._label_feats.norm(dim=-1, keepdim=True)
        self._labels = labels

        self.get_logger().info(f'Semantic map: {len(labels)} locations — {labels}')

    # ── Query ──────────────────────────────────────────────────────────────── #

    def query(self, text: str):
        """Return (best_label, score, position_dict) for a natural language query."""
        with torch.no_grad():
            tok  = clip.tokenize([text]).to(self.device)
            feat = self.model.encode_text(tok)
            feat = feat / feat.norm(dim=-1, keepdim=True)
            sims = (feat @ self._label_feats.T).squeeze(0).cpu().numpy()

        best_idx   = int(np.argmax(sims))
        best_label = self._labels[best_idx]
        best_score = float(sims[best_idx])
        position   = self._map[best_label]['position']
        return best_label, best_score, position

    # ── Navigate ───────────────────────────────────────────────────────────── #

    def navigate_to(self, x: float, y: float, label: str = ''):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0   # face forward (no specific heading)
        goal.pose = pose

        self.get_logger().info(f'Navigating to "{label}" at ({x:.2f}, {y:.2f}) ...')
        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return False

        result_future = goal_handle.get_result_async()
        self.get_logger().info('Driving ... (Ctrl-C to cancel)')
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrived at "{label}"!')
            return True
        else:
            self.get_logger().warn(f'Navigation ended with status {status}')
            return False

    # ── Interactive loop ────────────────────────────────────────────────────── #

    def run_interactive(self):
        print('\n── Semantic Navigation ──────────────────────────────────')
        print(f'Known locations: {self._labels}')
        print('Type a room or object name. Ctrl-C to quit.\n')

        while rclpy.ok():
            try:
                query = input('Where to go? ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not query:
                continue

            label, score, pos = self.query(query)
            print(f'  → Best match: "{label}"  (CLIP score={score:.3f})  '
                  f'pos=({pos["x"]:.2f}, {pos["y"]:.2f})')

            if score < 0.15:
                print('  ✗ Score too low — location may not have been seen during exploration.')
                continue

            confirm = input(f'  Navigate to "{label}"? [Y/n] ').strip().lower()
            if confirm in ('', 'y', 'yes'):
                self.navigate_to(pos['x'], pos['y'], label)
            else:
                print('  Cancelled.')


def main(args=None):
    rclpy.init(args=args)

    # Optional: single-shot CLI arg — ros2 run vlm_nav semantic_nav "kitchen"
    query_arg = None
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        query_arg = ' '.join(sys.argv[1:])

    node = SemanticNavNode()

    try:
        if query_arg:
            label, score, pos = node.query(query_arg)
            node.get_logger().info(f'Query: "{query_arg}" → "{label}" (score={score:.3f})')
            node.navigate_to(pos['x'], pos['y'], label)
        else:
            node.run_interactive()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
