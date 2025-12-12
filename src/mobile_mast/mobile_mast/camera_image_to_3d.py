# file: mobile_mast/camera_image_to_3d.py

import os
import math
from typing import Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Vector3

from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber


class CameraImageTo3DNode(Node):
    """
    Node: camera_image_to_3d

    Inputs:
      - detections_2d_topic: vision_msgs/Detection2DArray
      - lidar_topic        : sensor_msgs/PointCloud2

    Uses:
      - camera intrinsics (bullet1_intrinsics.yaml)
      - lidar_to_camera transform (lidar_to_bullet1.yaml)

    Output:
      - detections_3d_topic: vision_msgs/Detection3DArray
    """

    def __init__(self):
        super().__init__('camera_image_to_3d')

        # ------------- Parameters -------------
        self.declare_parameter('detections_topic', 'detections_2d')
        self.declare_parameter('lidar_topic', '/mast/orin1/lidar/foreground')
        self.declare_parameter('output_topic', 'detections_3d')

        # calibration file paths
        self.declare_parameter('intrinsics_file', '')
        self.declare_parameter('extrinsics_file', '')

        # thresholds
        self.declare_parameter('min_points_per_box', 20)

        # Read params
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        self.extrinsics_file = self.get_parameter('extrinsics_file').get_parameter_value().string_value
        self.min_points_per_box = self.get_parameter('min_points_per_box').get_parameter_value().integer_value

        if not self.intrinsics_file or not os.path.isfile(self.intrinsics_file):
            self.get_logger().error(f"Invalid intrinsics_file: {self.intrinsics_file}")
            raise RuntimeError("intrinsics_file missing")

        if not self.extrinsics_file or not os.path.isfile(self.extrinsics_file):
            self.get_logger().error(f"Invalid extrinsics_file: {self.extrinsics_file}")
            raise RuntimeError("extrinsics_file missing")

        self.get_logger().info(f"[CAM_3D] detections_topic : {self.detections_topic}")
        self.get_logger().info(f"[CAM_3D] lidar_topic      : {self.lidar_topic}")
        self.get_logger().info(f"[CAM_3D] output_topic     : {self.output_topic}")
        self.get_logger().info(f"[CAM_3D] intrinsics_file  : {self.intrinsics_file}")
        self.get_logger().info(f"[CAM_3D] extrinsics_file  : {self.extrinsics_file}")

        # ------------- Load calibration -------------
        self.K, self.image_width, self.image_height = self.load_intrinsics(self.intrinsics_file)
        self.T_lidar_cam = self.load_extrinsics(self.extrinsics_file)

        # ------------- ROS I/O -------------
        qos = QoSProfile(depth=10)

        # message_filters subscribers
        self.det_sub = MFSubscriber(self, Detection2DArray, self.detections_topic, qos_profile=qos)
        self.lidar_sub = MFSubscriber(self, PointCloud2, self.lidar_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.det_sub, self.lidar_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        # Publisher
        self.pub_3d = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            qos
        )

    # ------------- Calibration loading -------------

    def load_intrinsics(self, path: str) -> Tuple[np.ndarray, int, int]:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        w = int(data.get('image_width', 1920))
        h = int(data.get('image_height', 1080))
        cm = data['camera_matrix']['data']  # list of 9 elements

        K = np.array(cm, dtype=np.float32).reshape(3, 3)
        self.get_logger().info(f"[CAM_3D] Loaded intrinsics, width={w}, height={h}")
        return K, w, h

    def load_extrinsics(self, path: str) -> np.ndarray:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        T_data = data['T_lidar_to_camera']['data']
        T = np.array(T_data, dtype=np.float32).reshape(4, 4)
        self.get_logger().info(f"[CAM_3D] Loaded extrinsics:\n{T}")
        return T

    # ------------- Core callback -------------

    def synced_callback(self, det_msg: Detection2DArray, cloud_msg: PointCloud2):
        """
        Called when we have (detections_2d, lidar_cloud) with approximately same stamp.
        """
        # Convert LiDAR cloud to numpy in LiDAR frame
        points_lidar = self.pointcloud2_to_xyz(cloud_msg)
        if points_lidar.shape[0] == 0:
            return

        # Transform to camera frame
        points_cam = self.transform_lidar_to_camera(points_lidar)

        # Project to image plane
        uv, valid_mask = self.project_to_image(points_cam)
        points_cam = points_cam[valid_mask]
        uv = uv[valid_mask]

        # Build Detection3DArray
        det3d_array = Detection3DArray()
        det3d_array.header = det_msg.header  # same timestamp / frame as detections

        for det in det_msg.detections:
            # extract bbox in pixel coordinates
            # vision_msgs Detection2D bbox center is (x,y), size is (width,height)
            cx = det.bbox.center.x
            cy = det.bbox.center.y
            w = det.bbox.size_x
            h = det.bbox.size_y

            xmin = cx - w / 2.0
            ymin = cy - h / 2.0
            xmax = cx + w / 2.0
            ymax = cy + h / 2.0

            # find lidar points inside this box
            u = uv[:, 0]
            v = uv[:, 1]

            mask = (u >= xmin) & (u <= xmax) & (v >= ymin) & (v <= ymax)
            pts_in_box = points_cam[mask]

            if pts_in_box.shape[0] < self.min_points_per_box:
                # Not enough LiDAR support, skip this detection
                continue

            det3d = self.build_detection3d(det, pts_in_box)
            det3d_array.detections.append(det3d)

        if det3d_array.detections:
            self.pub_3d.publish(det3d_array)

    # ------------- Helper: PointCloud2 -> xyz -------------

    def pointcloud2_to_xyz(self, cloud_msg: PointCloud2) -> np.ndarray:
        points = pc2.read_points_list(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.array(points, dtype=np.float32)

    # ------------- Helper: LiDAR -> Camera frame -------------

    def transform_lidar_to_camera(self, pts_lidar: np.ndarray) -> np.ndarray:
        """
        pts_lidar: Nx3 array in LiDAR frame
        returns: Nx3 array in camera frame
        """
        N = pts_lidar.shape[0]
        pts_h = np.hstack([pts_lidar, np.ones((N, 1), dtype=np.float32)])  # Nx4
        pts_cam_h = (self.T_lidar_cam @ pts_h.T).T  # Nx4
        pts_cam = pts_cam_h[:, :3]
        return pts_cam

    # ------------- Helper: Project to image -------------

    def project_to_image(self, pts_cam: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        pts_cam: Nx3 array (X,Y,Z) in camera frame
        returns:
          uv: Nx2 pixel coordinates
          mask_valid: Nx bool
        """
        X = pts_cam[:, 0]
        Y = pts_cam[:, 1]
        Z = pts_cam[:, 2]

        # must be in front of camera
        mask_z = Z > 0.1
        X = X[mask_z]
        Y = Y[mask_z]
        Z = Z[mask_z]

        if X.size == 0:
            return np.zeros((0, 2), dtype=np.float32), np.zeros((pts_cam.shape[0],), dtype=bool)

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy

        uv = np.stack([u, v], axis=1)

        # filter to be within image bounds (optional but good)
        in_bounds = (
            (u >= 0) & (u < self.image_width) &
            (v >= 0) & (v < self.image_height)
        )

        # Build mask for original pts_cam indices
        mask_valid = np.zeros((pts_cam.shape[0],), dtype=bool)
        idxs = np.where(mask_z)[0]
        mask_valid[idxs[in_bounds]] = True

        return uv[in_bounds], mask_valid

    # ------------- Helper: Build Detection3D -------------

    def build_detection3d(self, det2d, pts_cam: np.ndarray) -> Detection3D:
        """
        pts_cam: Nx3 points in camera frame belonging to one 2D bbox.
        Uses percentiles to be robust against outliers.
        """
        X = pts_cam[:, 0]
        Y = pts_cam[:, 1]
        Z = pts_cam[:, 2]

        # Use percentiles (10th and 90th) to ignore outliers
        x_min, x_max = np.percentile(X, [10, 90])
        y_min, y_max = np.percentile(Y, [10, 90])
        z_min, z_max = np.percentile(Z, [10, 90])

        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0
        cz = (z_min + z_max) / 2.0

        sx = max(x_max - x_min, 0.1)
        sy = max(y_max - y_min, 0.1)
        sz = max(z_max - z_min, 0.1)

        det3d = Detection3D()
        det3d.header = det2d.header  # reuse same header as 2D if available

        # copy class info from 2D
        if det2d.results:
            det3d.results.append(det3d.results.__class__())
            det3d.results[0].hypothesis.class_id = det2d.results[0].hypothesis.class_id
            det3d.results[0].hypothesis.score = det2d.results[0].hypothesis.score

        # bbox center
        det3d.bbox.center = Pose()
        det3d.bbox.center.position.x = float(cx)
        det3d.bbox.center.position.y = float(cy)
        det3d.bbox.center.position.z = float(cz)
        det3d.bbox.center.orientation.w = 1.0  # no rotation for now

        # bbox size
        det3d.bbox.size = Vector3()
        det3d.bbox.size.x = float(sx)
        det3d.bbox.size.y = float(sy)
        det3d.bbox.size.z = float(sz)

        return det3d


def main(args=None):
    rclpy.init(args=args)
    node = CameraImageTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
