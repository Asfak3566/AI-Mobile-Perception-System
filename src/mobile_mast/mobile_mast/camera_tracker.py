# file: mobile_mast/camera_tracker.py

import os
import csv
from typing import Dict, List, Tuple

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import mahalanobis

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import CompressedImage
from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber

# CNN / ReID
import cv2
import torch
import torchvision.transforms as T
from torchvision.models import resnet18, ResNet18_Weights


class ReIDModel:
    """
    CNN Feature Extractor for Re-Identification.
    Uses ResNet18.
    """
    def __init__(self, use_cuda=True):
        self.device = torch.device('cuda' if torch.cuda.is_available() and use_cuda else 'cpu')
        print(f"[ReID] Using device: {self.device}")
        
        # Load Pretrained ResNet18
        self.model = resnet18(weights=ResNet18_Weights.DEFAULT)
        # Remove final FC layer to get features (512-dim)
        # ResNet: (conv1 -> bn1 -> relu -> maxpool -> layer1..4 -> avgpool -> fc)
        # We want output of avgpool: (Batch, 512, 1, 1) -> flatten -> (Batch, 512)
        self.model.fc = torch.nn.Identity()
        
        self.model.to(self.device)
        self.model.eval()
        
        self.transform = T.Compose([
            T.ToPILImage(),
            T.Resize((128, 64)), # Typical ReID size (H, W)
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        
    def extract(self, img_bgr: np.ndarray) -> np.ndarray:
        if img_bgr is None or img_bgr.size == 0:
            return np.zeros(512, dtype=np.float32)
            
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        tensor = self.transform(img_rgb).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            feat = self.model(tensor) # (1, 512)
            
        return feat.cpu().numpy().flatten()


class Track:
    """
    3D Constant Acceleration EKF Track.
    State x = [px, py, pz, vx, vy, vz, ax, ay, az]^T
    """

    def __init__(self, track_id: int, initial_pos: np.ndarray, timestamp: float, params: dict, feature: np.ndarray = None):
        self.id = track_id
        self.params = params

        # State: position + velocity + acceleration (9x1)
        self.x = np.zeros((9, 1), dtype=np.float32)
        self.x[0:3, 0] = initial_pos  # px, py, pz

        # Covariance (9x9)
        self.P = np.eye(9, dtype=np.float32) * 10.0
        # High uncertainty for initial velocity and acceleration
        self.P[3:6, 3:6] *= 10.0
        self.P[6:9, 6:9] *= 100.0

        # Last update time
        self.last_update = timestamp
        self.age = 0.0
        self.hits = 1  # how many times we've been updated
        self.hit_streak = 1
        self.time_since_update = 0

        # Feature for ReID (Moving Average)
        self.feature = feature
        if self.feature is not None:
            self.feature = self.feature / np.linalg.norm(self.feature)

        # Measurement matrix: we measure position only (3x9)
        self.H = np.zeros((3, 9), dtype=np.float32)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # Measurement noise R
        r = params.get('measurement_noise_pos', 2.0)
        self.R = np.eye(3, dtype=np.float32) * (r ** 2)

    def predict(self, t: float):
        """Predict state forward to time t using constant acceleration."""
        dt = t - self.last_update
        if dt <= 0.0:
            return

        self.time_since_update += dt
        self.age += dt

        # Transition Matrix F (9x9) for Constant Acceleration
        # p = p + v*dt + 0.5*a*dt^2
        # v = v + a*dt
        # a = a
        F = np.eye(9, dtype=np.float32)
        
        # Position updates
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[0, 6] = 0.5 * dt**2
        F[1, 7] = 0.5 * dt**2
        F[2, 8] = 0.5 * dt**2
        
        # Velocity updates
        F[3, 6] = dt
        F[4, 7] = dt
        F[5, 8] = dt

        # Process Noise Q (9x9)
        q_pos = self.params.get('process_noise_pos', 0.1)
        q_vel = self.params.get('process_noise_vel', 0.1)
        q_acc = self.params.get('process_noise_acc', 1.0)
        
        Q = np.zeros((9, 9), dtype=np.float32)
        # Diagonal approximation for simplicity
        Q[0, 0] = q_pos
        Q[1, 1] = q_pos
        Q[2, 2] = q_pos
        Q[3, 3] = q_vel
        Q[4, 4] = q_vel
        Q[5, 5] = q_vel
        Q[6, 6] = q_acc
        Q[7, 7] = q_acc
        Q[8, 8] = q_acc

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        self.last_update = t

    def update(self, z: np.ndarray, t: float, feature: np.ndarray = None):
        """Update with position measurement z (3x1)."""
        # Note: predict() is usually called before this in the loop
        
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(9, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P

        # Update feature (EMA)
        if feature is not None and self.feature is not None:
            alpha = 0.9
            norm_feat = feature / np.linalg.norm(feature)
            self.feature = alpha * self.feature + (1 - alpha) * norm_feat
            self.feature /= np.linalg.norm(self.feature)
        elif self.feature is None and feature is not None:
            self.feature = feature / np.linalg.norm(feature)

        self.hits += 1
        self.hit_streak += 1
        self.time_since_update = 0
        self.last_update = t

    def get_position(self) -> np.ndarray:
        return self.x[0:3, 0]

    def is_confirmed(self, min_hits: int) -> bool:
        return self.hits >= min_hits

    def mahalanobis_distance(self, z: np.ndarray) -> float:
        """Calculate Mahalanobis distance between measurement z and track state."""
        S = self.H @ self.P @ self.H.T + self.R
        y = z - self.H @ self.x
        try:
            inv_S = np.linalg.inv(S)
            d_squared = y.T @ inv_S @ y
            return float(np.sqrt(d_squared))
        except np.linalg.LinAlgError:
            return 1e6
        
    def feature_distance(self, feature: np.ndarray) -> float:
        """Cosine distance: 1 - cosine_similarity"""
        if self.feature is None or feature is None:
            return 1.0 # Max distance if no features
        
        # Assume features are normalized
        f1 = self.feature
        f2 = feature / np.linalg.norm(feature)
        
        dot = np.dot(f1, f2)
        return float(1.0 - dot)


class CameraTrackerNode(Node):
    """
    EKF-based multi-object tracker for 3D camera detections.
    Uses Hungarian Algorithm and Mahalanobis Distance.
    """

    def __init__(self):
        super().__init__('camera_tracker')

        # One main parameter: path to tracker YAML
        self.declare_parameter('tracker_config_file', '')

        cfg_path = self.get_parameter('tracker_config_file').get_parameter_value().string_value
        if not cfg_path or not os.path.isfile(cfg_path):
            self.get_logger().error(f"[TRACKER] Invalid tracker_config_file: {cfg_path}")
            raise RuntimeError("tracker_config_file missing or invalid")

        self.get_logger().info(f"[TRACKER] Using config: {cfg_path}")

        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # Topics
        self.detections_topic = cfg.get('detections_topic', '/mast/orin1/camera/detections_3d')
        self.tracks_topic = cfg.get('tracks_topic', '/mast/orin1/camera/tracks_3d')
        self.image_topic = 'image_raw/compressed' # Hardcoded or from config?

        # Tracker parameters
        self.max_age = float(cfg.get('max_age', 1.0))
        self.min_hits = int(cfg.get('min_hits', 3))
        self.max_assoc_dist = float(cfg.get('max_association_distance', 5.0)) # Gating threshold (Mahalanobis or Euclidean)
        self.use_mahalanobis = True
        
        # ReID weight
        # If 0.0, use only Mahalanobis. If 1.0, use only ReID.
        self.reid_weight = 0.5 

        self.params = {
            'process_noise_pos': float(cfg.get('process_noise_pos', 1.0)),
            'process_noise_vel': float(cfg.get('process_noise_vel', 1.0)),
            'process_noise_acc': float(cfg.get('process_noise_acc', 1.0)),
            'measurement_noise_pos': float(cfg.get('measurement_noise_pos', 2.0)),
        }

        # Region & angles
        cfg_dir = os.path.dirname(cfg_path)
        region_file = cfg.get('region_file', 'region_camera.csv')
        angles_file = cfg.get('initial_angles_file', 'initial_angles_camera.csv')

        self.region = self.load_region(os.path.join(cfg_dir, region_file))
        self.initial_yaw_deg = self.load_initial_yaw(os.path.join(cfg_dir, angles_file))

        self.get_logger().info(f"[TRACKER] detections_topic : {self.detections_topic}")
        self.get_logger().info(f"[TRACKER] tracks_topic     : {self.tracks_topic}")
        self.get_logger().info(f"[TRACKER] region           : {self.region}")
        self.get_logger().info(f"[TRACKER] max_age={self.max_age}, min_hits={self.min_hits}, max_assoc_dist={self.max_assoc_dist}")

        # Tracks state
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1
        
        # Init ReID
        try:
            self.reid_model = ReIDModel(use_cuda=True)
            self.get_logger().info("[TRACKER] ReID Model initialized")
        except Exception as e:
            self.get_logger().error(f"[TRACKER] Failed to init ReID: {e}")
            self.reid_model = None

        qos = QoSProfile(depth=10)

        # Subscribers (Sync)
        self.sub_det = MFSubscriber(self, Detection3DArray, self.detections_topic, qos_profile=qos)
        self.sub_img = MFSubscriber(self, CompressedImage, self.image_topic, qos_profile=qos)
        
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_det, self.sub_img],
            queue_size=10,
            slop=0.2
        )
        self.sync.registerCallback(self.detections_callback)

        # Publisher
        self.pub = self.create_publisher(
            Detection3DArray,
            self.tracks_topic,
            qos,
        )

    # ---------- Region loading ----------

    def load_region(self, path: str) -> Tuple[float, float, float, float, float, float]:
        if not os.path.isfile(path):
            self.get_logger().warn(f"[TRACKER] Region file not found: {path}, using no region limits.")
            return (-1e6, 1e6, -1e6, 1e6, -1e6, 1e6)

        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                vals = [float(v.strip()) for v in row]
                if len(vals) != 6:
                    continue
                return tuple(vals)

        self.get_logger().warn(f"[TRACKER] Could not parse region file: {path}, using no limits.")
        return (-1e6, 1e6, -1e6, 1e6, -1e6, 1e6)

    # ---------- Angles ----------

    def load_initial_yaw(self, path: str) -> float:
        if not os.path.isfile(path):
            return 0.0

        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                try:
                    return float(row[0])
                except ValueError:
                    continue
        return 0.0

    # ---------- Main callback ----------

    def detections_callback(self, msg: Detection3DArray, img_msg: CompressedImage):
        t_now = self.get_clock().now().nanoseconds / 1e9

        # Decode image
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 1) Predict all tracks forward
        for tr in self.tracks.values():
            tr.predict(t_now)

        # 2) Filter detections inside region & Extract Features
        det_positions: List[np.ndarray] = []
        det_features: List[np.ndarray] = []
        det_objects: List[Detection3D] = []

        for det in msg.detections:
            px = det.bbox.center.position.x
            py = det.bbox.center.position.y
            pz = det.bbox.center.position.z

            if not self.is_in_region(px, py, pz):
                continue

            det_positions.append(np.array([px, py, pz], dtype=np.float32).reshape(3, 1))
            det_objects.append(det)
            
            # Extract feature
            feat = np.zeros(512, dtype=np.float32)
            if self.reid_model and cv_img is not None:
                # Decode bbox info from class_id
                cid = det.results[0].hypothesis.class_id
                if '|' in cid:
                    try:
                        _, bbox_part = cid.split('|')
                        bx, by, bw, bh = map(float, bbox_part.split(','))
                        
                        # Crop image (clamp to bounds)
                        h, w, _ = cv_img.shape
                        x1 = int(bx - bw/2)
                        y1 = int(by - bh/2)
                        x2 = int(bx + bw/2)
                        y2 = int(by + bh/2)
                        
                        x1 = max(0, x1); y1 = max(0, y1)
                        x2 = min(w, x2); y2 = min(h, y2)
                        
                        if x2 > x1 and y2 > y1:
                            crop = cv_img[y1:y2, x1:x2]
                            feat = self.reid_model.extract(crop)
                    except Exception as e:
                        # self.get_logger().warn(f"Failed to parse bbox: {e}")
                        pass
            
            det_features.append(feat)

        # 3) Associate detections to tracks
        assigned_det_indices, assigned_track_ids, unassigned_dets, unassigned_tracks = \
            self.associate_detections_to_tracks(det_positions, det_features)

        # 4) Update assigned tracks
        for det_idx, track_id in zip(assigned_det_indices, assigned_track_ids):
            pos = det_positions[det_idx]
            feat = det_features[det_idx]
            
            # Check if this feature is "empty" (all zeros), if so don't update feature?
            # ReIDModel returns zeros if update fails.
            if np.all(feat == 0):
                feat = None
                
            self.tracks[track_id].update(pos, t_now, feat)

        # 5) Create new tracks for unassigned detections
        for det_idx in unassigned_dets:
            pos = det_positions[det_idx]
            feat = det_features[det_idx]
            
            if np.all(feat == 0):
                feat = None
                
            self.create_track(pos, t_now, feat)

        # 6) Remove stale tracks
        self.prune_tracks(t_now)

        # 7) Publish confirmed tracks
        out = Detection3DArray()
        out.header = msg.header

        for track_id, track in self.tracks.items():
            if not track.is_confirmed(self.min_hits):
                continue
            pos = track.get_position()
            det3d = self.track_to_detection(track_id, pos)
            out.detections.append(det3d)

        if out.detections:
            self.pub.publish(out)

    # ---------- Helpers ----------

    def is_in_region(self, x: float, y: float, z: float) -> bool:
        min_x, max_x, min_y, max_y, min_z, max_z = self.region
        return (min_x <= x <= max_x and
                min_y <= y <= max_y and
                min_z <= z <= max_z)

    def create_track(self, pos: np.ndarray, t: float, feature: np.ndarray = None):
        track_id = self.next_track_id
        self.next_track_id += 1

        self.tracks[track_id] = Track(track_id, pos, t, self.params, feature)
        self.get_logger().info(f"[TRACKER] New track {track_id} at {pos.flatten()}")

    def prune_tracks(self, t_now: float):
        to_delete = []
        for track_id, track in self.tracks.items():
            if track.time_since_update > self.max_age:
                to_delete.append(track_id)

        for tid in to_delete:
            self.get_logger().info(f"[TRACKER] Removing track {tid} (stale)")
            del self.tracks[tid]

    def associate_detections_to_tracks(self, detections: List[np.ndarray], features: List[np.ndarray]):
        """
        Optimal association using Hungarian Algorithm (Munkres).
        Uses combination of Mahalanobis and Cosine distance.
        """
        track_ids = list(self.tracks.keys())
        N_t = len(track_ids)
        N_d = len(detections)

        if N_t == 0:
            return [], [], list(range(N_d)), []
        if N_d == 0:
            return [], [], [], track_ids

        # Cost matrix
        cost_matrix = np.zeros((N_t, N_d), dtype=np.float32)
        
        for i, tid in enumerate(track_ids):
            track = self.tracks[tid]
            for j, det_pos in enumerate(detections):
                # Spatial cost (Mahalanobis)
                if self.use_mahalanobis:
                    spatial_cost = track.mahalanobis_distance(det_pos)
                else:
                    spatial_cost = np.linalg.norm(track.get_position() - det_pos.flatten())
                
                # Appearance cost (Cosine)
                feat_cost = 1.0
                det_feat = features[j]
                if not np.all(det_feat == 0):
                    feat_cost = track.feature_distance(det_feat)
                    
                # Combined cost
                # Normalize spatial cost to be vaguely around [0, 1] range is hard without threshold.
                # Assuming spatial gaiting is done later or we trust raw values?
                # Mahalanobis is often < 5-10 for match. Cosine is < 0.2-0.5.
                # Let's just sum them with weight.
                
                total_cost = (1.0 - self.reid_weight) * spatial_cost + \
                             (self.reid_weight * feat_cost * 10.0) # Boost feature cost to match scale
                             
                cost_matrix[i, j] = total_cost

        # Hungarian Algorithm
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        assigned_dets = []
        assigned_tracks = []
        unassigned_dets = set(range(N_d))
        unassigned_tracks = set(range(N_t))

        for r, c in zip(row_ind, col_ind):
            # Apply gating
            # We use a relaxed spatial gating for now, because cost is mixed.
            # But strictly speaking we should probably gate separately.
            
            # Simple check: if spatial distance is HUGE, reject.
            track = self.tracks[track_ids[r]]
            if self.use_mahalanobis:
                d_spatial = track.mahalanobis_distance(detections[c])
            else:
                d_spatial = np.linalg.norm(track.get_position() - detections[c].flatten())
                
            if d_spatial <= self.max_assoc_dist * 2.0: # Relaxed gating
                assigned_tracks.append(track_ids[r])
                assigned_dets.append(c)
                unassigned_tracks.discard(r)
                unassigned_dets.discard(c)

        unassigned_track_ids = [track_ids[i] for i in unassigned_tracks]
        return assigned_dets, assigned_tracks, list(unassigned_dets), unassigned_track_ids

    def track_to_detection(self, track_id: int, pos: np.ndarray) -> Detection3D:
        det = Detection3D()
        det.header.stamp = self.get_clock().now().to_msg()
        
        det.results.append(det.results.__class__())
        det.results[0].hypothesis.class_id = f"track_{track_id}"
        det.results[0].hypothesis.score = 1.0

        det.bbox.center = Pose()
        det.bbox.center.position.x = float(pos[0])
        det.bbox.center.position.y = float(pos[1])
        det.bbox.center.position.z = float(pos[2])
        det.bbox.center.orientation.w = 1.0

        det.bbox.size = Vector3()
        det.bbox.size.x = 1.0
        det.bbox.size.y = 1.0
        det.bbox.size.z = 1.0

        return det


def main(args=None):
    rclpy.init(args=args)
    node = CameraTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

