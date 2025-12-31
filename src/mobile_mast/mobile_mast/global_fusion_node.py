# file: mobile_mast/global_fusion_node.py

import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import yaml
import numpy as np
from scipy.optimize import linear_sum_assignment

from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Vector3


class GlobalTrack:
    """
    Simple persistent global track.
    State x = [px, py, pz]^T (Position only for now, could be EKF)
    """
    def __init__(self, track_id: int, pos: np.ndarray, timestamp: float):
        self.id = track_id
        self.pos = pos
        self.last_update = timestamp
        self.hits = 1
        self.sources = set()

    def update(self, pos: np.ndarray, timestamp: float, source: str):
        # Simple weighted update (could be Kalman Filter)
        alpha = 0.7 # Trust new measurement more
        self.pos = self.pos * (1 - alpha) + pos * alpha
        self.last_update = timestamp
        self.hits += 1
        self.sources.add(source)

    def predict(self, timestamp: float):
        # Static prediction for now
        pass


class GlobalFusionNode(Node):
    """
    Global Fusion Node -> Global Tracker.
    Maintains persistent tracks from multiple sources.
    """

    def __init__(self):
        super().__init__('global_fusion_node')

        # Parameter: path to global fusion config
        self.declare_parameter('fusion_config_file', '')

        cfg_path = self.get_parameter('fusion_config_file').get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().error("[FUSION] fusion_config_file parameter is empty")
            raise RuntimeError("fusion_config_file must be provided")

        self.get_logger().info(f"[FUSION] Using config: {cfg_path}")

        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # Read config
        self.input_topics: List[str] = cfg.get('input_topics', [])
        self.output_topic: str = cfg.get('output_topic', '/mast/global/tracks_3d')

        self.fusion_distance: float = float(cfg.get('fusion_distance', 3.0))
        self.max_age_sec: float = float(cfg.get('max_age_sec', 0.5))
        self.fusion_rate_hz: float = float(cfg.get('fusion_rate_hz', 10.0))

        if not self.input_topics:
            self.get_logger().error("[FUSION] No input_topics provided in config")
            raise RuntimeError("No input_topics in fusion config")

        self.get_logger().info(f"[FUSION] Input topics  : {self.input_topics}")
        self.get_logger().info(f"[FUSION] Output topic  : {self.output_topic}")
        self.get_logger().info(f"[FUSION] fusion_dist   : {self.fusion_distance}")
        self.get_logger().info(f"[FUSION] max_age_sec   : {self.max_age_sec}")
        self.get_logger().info(f"[FUSION] fusion_rate   : {self.fusion_rate_hz} Hz")

        qos = QoSProfile(depth=10)

        # Cache for latest messages per topic
        self.latest_msgs: Dict[str, Tuple[Detection3DArray, float]] = {}

        # Global Tracks
        self.tracks: Dict[int, GlobalTrack] = {}
        self.next_track_id = 1

        # Subscribers
        self.subs = []
        for topic in self.input_topics:
            self.get_logger().info(f"[FUSION] Subscribing to {topic}")
            sub = self.create_subscription(
                Detection3DArray,
                topic,
                lambda msg, t=topic: self._callback(msg, t),
                qos
            )
            self.subs.append(sub)

        # Publisher
        self.pub = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            qos
        )

        # Timer for periodic fusion
        period = 1.0 / self.fusion_rate_hz if self.fusion_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self.fuse_step)

    def _callback(self, msg: Detection3DArray, topic: str):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.latest_msgs[topic] = (msg, now_sec)

    def fuse_step(self):
        """
        Periodically called:
        1. Collect all valid detections from recent messages.
        2. Associate them to existing global tracks (Hungarian).
        3. Update tracks / Create new tracks.
        4. Prune stale tracks.
        5. Publish.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # 1. Collect all detections
        all_detections: List[np.ndarray] = []
        all_sources: List[str] = []

        for topic, (msg, t_recv) in self.latest_msgs.items():
            if (now_sec - t_recv) > self.max_age_sec:
                continue

            for det in msg.detections:
                px = det.bbox.center.position.x
                py = det.bbox.center.position.y
                pz = det.bbox.center.position.z
                all_detections.append(np.array([px, py, pz], dtype=np.float32))
                all_sources.append(topic)
        
        # 2. Association
        if not self.tracks and not all_detections:
            return

        track_ids = list(self.tracks.keys())
        N_t = len(track_ids)
        N_d = len(all_detections)

        # Cost matrix
        cost = np.zeros((N_t, N_d), dtype=np.float32)
        for i, tid in enumerate(track_ids):
            t_pos = self.tracks[tid].pos
            for j, d_pos in enumerate(all_detections):
                cost[i, j] = np.linalg.norm(t_pos - d_pos)

        # Hungarian Algorithm
        if N_t > 0 and N_d > 0:
            row_ind, col_ind = linear_sum_assignment(cost)
        else:
            row_ind, col_ind = [], []

        assigned_tracks = set()
        assigned_dets = set()

        # 3. Update Tracks
        for r, c in zip(row_ind, col_ind):
            if cost[r, c] <= self.fusion_distance:
                tid = track_ids[r]
                self.tracks[tid].update(all_detections[c], now_sec, all_sources[c])
                assigned_tracks.add(tid)
                assigned_dets.add(c)

        # Create new tracks for unassigned detections
        for j in range(N_d):
            if j not in assigned_dets:
                self.create_track(all_detections[j], now_sec, all_sources[j])

        # 4. Prune Stale Tracks
        to_delete = []
        for tid, track in self.tracks.items():
            if (now_sec - track.last_update) > self.max_age_sec:
                to_delete.append(tid)
        
        for tid in to_delete:
            del self.tracks[tid]

        # 5. Publish
        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map" # Or whatever global frame

        for tid, track in self.tracks.items():
            det = Detection3D()
            det.header = out.header

            det.results.append(det.results.__class__())
            det.results[0].hypothesis.class_id = f"global_{tid}"
            det.results[0].hypothesis.score = 1.0

            det.bbox.center = Pose()
            det.bbox.center.position.x = float(track.pos[0])
            det.bbox.center.position.y = float(track.pos[1])
            det.bbox.center.position.z = float(track.pos[2])
            det.bbox.center.orientation.w = 1.0

            det.bbox.size = Vector3()
            det.bbox.size.x = 1.5
            det.bbox.size.y = 1.5
            det.bbox.size.z = 1.5

            out.detections.append(det)

        self.pub.publish(out)

    def create_track(self, pos: np.ndarray, timestamp: float, source: str):
        tid = self.next_track_id
        self.next_track_id += 1
        new_track = GlobalTrack(tid, pos, timestamp)
        new_track.sources.add(source)
        self.tracks[tid] = new_track


def main(args=None):
    rclpy.init(args=args)
    node = GlobalFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
