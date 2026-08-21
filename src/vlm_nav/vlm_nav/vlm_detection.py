#!/usr/bin/env python3
"""
VLM Detection Node — runs during frontier exploration.

Every PROCESS_EVERY_N frames:
  1. Faster R-CNN proposes bounding boxes in RGB frame
  2. CLIP scores each crop against TARGET_LABELS (rooms + objects)
  3. Best label stored in JSON with robot's current MAP pose

No depth sensor needed: robot pose ≈ object location (good enough for room-level nav).
JSON persists across runs — higher-confidence detections overwrite lower ones.
"""

import json
import math
import os
import time
from pathlib import Path

import clip
import cv2
import numpy as np
import rclpy
import torch
import torchvision
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# ── Config ────────────────────────────────────────────────────────────────── #

SEMANTIC_MAP_PATH   = str(Path.home() / 'dl_hackathon' / 'semantic_map.json')
PROCESS_EVERY_N     = 30          # process 1 frame every N (camera runs ~10 Hz)
CLIP_THRESHOLD      = 0.22        # min cosine similarity to record detection
RCNN_THRESHOLD      = 0.7         # min Faster R-CNN objectness score
MAX_BOXES           = 6           # max bounding boxes to score per frame

TARGET_LABELS = [
    # Rooms / areas — what user will say
    "kitchen", "living room", "bedroom", "bathroom", "hallway",
    "dining room", "office", "entrance", "laundry room",
    # Objects that anchor a room
    "refrigerator", "sofa", "couch", "bed", "toilet", "sink",
    "bathtub", "television", "dining table", "chair", "desk",
    "bookshelf", "microwave", "oven", "washing machine",
    "fireplace", "staircase",
]

# ── Node ──────────────────────────────────────────────────────────────────── #

class VLMDetectionNode(Node):
    def __init__(self):
        super().__init__('vlm_detection')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'VLM running on {self.device}')

        # ── Load models ──
        self.get_logger().info('Loading CLIP ViT-B/32 ...')
        self.clip_model, self.clip_preprocess = clip.load('ViT-B/32', device=self.device)
        self.clip_model.eval()

        self.get_logger().info('Loading Faster R-CNN ...')
        self.rcnn = torchvision.models.detection.fasterrcnn_resnet50_fpn_v2(
            weights=torchvision.models.detection.FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT)
        self.rcnn.eval().to(self.device)

        # Pre-encode all label texts (fixed, do once)
        with torch.no_grad():
            tokens = clip.tokenize(TARGET_LABELS).to(self.device)
            self.label_feats = self.clip_model.encode_text(tokens)
            self.label_feats = self.label_feats / self.label_feats.norm(dim=-1, keepdim=True)

        # ── ROS ──
        self.bridge   = CvBridge()
        self.tf_buf   = Buffer()
        self.tf_lis   = TransformListener(self.tf_buf, self)
        self._frame_count = 0

        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self._annot_pub = self.create_publisher(Image, '/vlm/annotated_image', 10)

        # ── Semantic map ──
        self._map: dict = self._load_map()
        self.get_logger().info(f'Semantic map loaded: {len(self._map)} entries — {SEMANTIC_MAP_PATH}')
        self.get_logger().info('VLM detection ready. Watching /camera/image_raw ...')

    # ── Map persistence ────────────────────────────────────────────────────── #

    def _load_map(self) -> dict:
        if os.path.exists(SEMANTIC_MAP_PATH):
            with open(SEMANTIC_MAP_PATH) as f:
                return json.load(f)
        return {}

    def _save_map(self):
        os.makedirs(os.path.dirname(SEMANTIC_MAP_PATH), exist_ok=True)
        with open(SEMANTIC_MAP_PATH, 'w') as f:
            json.dump(self._map, f, indent=2)

    # ── Robot pose ─────────────────────────────────────────────────────────── #

    def _get_map_pose(self):
        """Return (x, y, yaw) of robot in map frame, or None."""
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint', Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ── Image callback ─────────────────────────────────────────────────────── #

    def _image_cb(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % PROCESS_EVERY_N != 0:
            return

        pose = self._get_map_pose()
        if pose is None:
            return                      # map not ready yet

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self._detect_and_store(rgb, pose)

    # ── Detection ──────────────────────────────────────────────────────────── #

    def _detect_and_store(self, rgb: np.ndarray, pose):
        h, w = rgb.shape[:2]
        rx, ry, _ = pose

        # Faster R-CNN boxes
        tensor = torchvision.transforms.functional.to_tensor(rgb).unsqueeze(0).to(self.device)
        with torch.no_grad():
            preds = self.rcnn(tensor)[0]

        boxes  = preds['boxes'].cpu().numpy()
        scores = preds['scores'].cpu().numpy()

        # Keep high-confidence boxes, limit to MAX_BOXES
        keep = scores >= RCNN_THRESHOLD
        boxes, scores = boxes[keep], scores[keep]
        if len(boxes) == 0:
            boxes  = np.array([[0, 0, w, h]], dtype=np.float32)
            scores = np.array([1.0])
        idx   = np.argsort(-scores)[:MAX_BOXES]
        boxes = boxes[idx]

        annot = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        updated = False
        for box in boxes:
            x1, y1, x2, y2 = box.astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            crop = rgb[y1:y2, x1:x2]
            label, score = self._clip_label(crop)

            color = (0, 200, 0) if score >= CLIP_THRESHOLD else (100, 100, 100)
            cv2.rectangle(annot, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annot, f'{label} {score:.2f}', (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            if score < CLIP_THRESHOLD:
                continue

            prev = self._map.get(label, {}).get('score', 0.0)
            if score > prev:
                self._map[label] = {
                    'score': float(score),
                    'position': {'x': float(rx), 'y': float(ry), 'z': 0.0},
                }
                self._save_map()
                self.get_logger().info(
                    f'[VLM] {label:<20} score={score:.3f}  pos=({rx:.2f}, {ry:.2f})')
                updated = True

        self._annot_pub.publish(self.bridge.cv2_to_imgmsg(annot, encoding='bgr8'))

    def _clip_label(self, crop: np.ndarray):
        """Return (label, cosine_score) for best matching label."""
        from PIL import Image as PILImage
        pil = PILImage.fromarray(crop)
        img_tensor = self.clip_preprocess(pil).unsqueeze(0).to(self.device)
        with torch.no_grad():
            img_feat = self.clip_model.encode_image(img_tensor)
            img_feat = img_feat / img_feat.norm(dim=-1, keepdim=True)
            sims     = (img_feat @ self.label_feats.T).squeeze(0)
        best_idx   = sims.argmax().item()
        best_score = sims[best_idx].item()
        return TARGET_LABELS[best_idx], best_score


def main(args=None):
    rclpy.init(args=args)
    node = VLMDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
