# file: mobile_mast/lidar_cluster_node.py

import math
from typing import List, Tuple

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from visualization_msgs.msg import Marker, MarkerArray


class LidarClusterNode(Node):
    """
    LiDAR clustering node.

    Pipeline:
      1. Subscribes to filtered LiDAR cloud (PointCloud2).
      2. Optional ground removal (z range).
      3. Optional ROI filter (x,y,z box).
      4. DBSCAN-based clustering (used for 'euclidean' and 'dbscan' modes).
      5. Publishes:
         - A cloud where intensity encodes cluster_id ("output_topic").
         - Optional 3D bounding boxes as MarkerArray ("bounding_boxes.publish_as").

    Parameters are loaded from lidar_cluster_params.yaml, e.g.:

      input_topic: "/mast/orin1/lidar/points_filtered"
      output_topic: "/mast/orin1/lidar/clusters"
      frame_id: "rslidar"
      ground_removal.enabled: true
      roi.enabled: false
      clustering.method: "euclidean"
      clustering.euclidean.cluster_tolerance: 0.5
      clustering.euclidean.min_cluster_size: 10
      clustering.euclidean.max_cluster_size: 20000
      bounding_boxes.enabled: true
      bounding_boxes.publish_as: "/mast/orin1/lidar/objects_3d"
    """

    def __init__(self):
        super().__init__('lidar_cluster_node')

        # ───────────── Declare parameters (with defaults) ─────────────
        # Top-level topics & frame
        self.declare_parameter('input_topic', '/mast/orin1/lidar/foreground')
        self.declare_parameter('output_topic', '/mast/orin1/lidar/clusters')
        self.declare_parameter('frame_id', 'rslidar')

        # Ground removal
        self.declare_parameter('ground_removal.enabled', True)
        self.declare_parameter('ground_removal.ground_z_min', -0.3)
        self.declare_parameter('ground_removal.ground_z_max', 0.3)

        # ROI
        self.declare_parameter('roi.enabled', False)
        self.declare_parameter('roi.min_x', -60.0)
        self.declare_parameter('roi.max_x', 60.0)
        self.declare_parameter('roi.min_y', -50.0)
        self.declare_parameter('roi.max_y', 50.0)
        self.declare_parameter('roi.min_z', -5.0)
        self.declare_parameter('roi.max_z', 5.0)

        # Clustering
        self.declare_parameter('clustering.method', 'euclidean')  # 'euclidean' or 'dbscan'

        # Euclidean-like clustering (we use DBSCAN underneath)
        self.declare_parameter('clustering.euclidean.cluster_tolerance', 0.5)
        self.declare_parameter('clustering.euclidean.min_cluster_size', 10)
        self.declare_parameter('clustering.euclidean.max_cluster_size', 20000)

        # DBSCAN (same backend but separate config if you want)
        self.declare_parameter('clustering.dbscan.eps', 0.5)
        self.declare_parameter('clustering.dbscan.min_points', 10)

        # Bounding boxes
        self.declare_parameter('bounding_boxes.enabled', True)
        self.declare_parameter('bounding_boxes.publish_as', '/mast/orin1/lidar/objects_3d')

        # Debug
        self.declare_parameter('debug.publish_filtered_cloud', False)
        self.declare_parameter('debug.filtered_cloud_topic', '/mast/orin1/lidar/filtered_debug')
        self.declare_parameter('debug.publish_cluster_clouds', False)

        # ───────────── Read parameters ─────────────
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.ground_enabled = self.get_parameter('ground_removal.enabled').get_parameter_value().bool_value
        self.ground_z_min = self.get_parameter('ground_removal.ground_z_min').get_parameter_value().double_value
        self.ground_z_max = self.get_parameter('ground_removal.ground_z_max').get_parameter_value().double_value

        self.roi_enabled = self.get_parameter('roi.enabled').get_parameter_value().bool_value
        self.roi_min_x = self.get_parameter('roi.min_x').get_parameter_value().double_value
        self.roi_max_x = self.get_parameter('roi.max_x').get_parameter_value().double_value
        self.roi_min_y = self.get_parameter('roi.min_y').get_parameter_value().double_value
        self.roi_max_y = self.get_parameter('roi.max_y').get_parameter_value().double_value
        self.roi_min_z = self.get_parameter('roi.min_z').get_parameter_value().double_value
        self.roi_max_z = self.get_parameter('roi.max_z').get_parameter_value().double_value

        self.cluster_method = self.get_parameter('clustering.method').get_parameter_value().string_value

        self.euclidean_tol = self.get_parameter(
            'clustering.euclidean.cluster_tolerance').get_parameter_value().double_value
        self.euclidean_min_size = self.get_parameter(
            'clustering.euclidean.min_cluster_size').get_parameter_value().integer_value
        self.euclidean_max_size = self.get_parameter(
            'clustering.euclidean.max_cluster_size').get_parameter_value().integer_value

        self.dbscan_eps = self.get_parameter('clustering.dbscan.eps').get_parameter_value().double_value
        self.dbscan_min_points = self.get_parameter('clustering.dbscan.min_points').get_parameter_value().integer_value

        self.bb_enabled = self.get_parameter('bounding_boxes.enabled').get_parameter_value().bool_value
        self.bb_topic = self.get_parameter('bounding_boxes.publish_as').get_parameter_value().string_value

        self.debug_filtered = self.get_parameter('debug.publish_filtered_cloud').get_parameter_value().bool_value
        self.debug_filtered_topic = self.get_parameter('debug.filtered_cloud_topic').get_parameter_value().string_value
        self.debug_cluster_clouds = self.get_parameter('debug.publish_cluster_clouds').get_parameter_value().bool_value

        # ───────────── Log configuration ─────────────
        self.get_logger().info(f"[LIDAR_CLUSTER] input_topic  : {self.input_topic}")
        self.get_logger().info(f"[LIDAR_CLUSTER] output_topic : {self.output_topic}")
        self.get_logger().info(f"[LIDAR_CLUSTER] frame_id     : {self.frame_id}")
        self.get_logger().info(f"[LIDAR_CLUSTER] ground_removal: {self.ground_enabled}, z in "
                               f"[{self.ground_z_min}, {self.ground_z_max}] will be removed")
        self.get_logger().info(f"[LIDAR_CLUSTER] roi_enabled  : {self.roi_enabled}")
        self.get_logger().info(f"[LIDAR_CLUSTER] clustering   : {self.cluster_method}")
        self.get_logger().info(f"[LIDAR_CLUSTER] euclidean tol: {self.euclidean_tol}, "
                               f"min_size: {self.euclidean_min_size}, max_size: {self.euclidean_max_size}")
        self.get_logger().info(f"[LIDAR_CLUSTER] bb_enabled   : {self.bb_enabled}, topic: {self.bb_topic}")

        # ───────────── ROS I/O ─────────────
        qos = QoSProfile(depth=10)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        self.cluster_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos
        )

        self.bb_pub = self.create_publisher(
            MarkerArray,
            self.bb_topic,
            qos
        ) if self.bb_enabled else None

        self.debug_filtered_pub = self.create_publisher(
            PointCloud2,
            self.debug_filtered_topic,
            qos
        ) if self.debug_filtered else None

    # ─────────────────────────────
    # Callback
    # ─────────────────────────────
    def cloud_callback(self, msg: PointCloud2):
        # Read points as list of (x, y, z, intensity)
        points = pc2.read_points_list(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        )
        if not points:
            return

        pts = np.array(points, dtype=np.float32)  # shape (N, 4)
        xyz = pts[:, :3]

        # 1) Ground removal
        if self.ground_enabled:
            z = xyz[:, 2]
            mask_ground = (z >= self.ground_z_min) & (z <= self.ground_z_max)
            mask = ~mask_ground
            xyz = xyz[mask]
            pts = pts[mask]

        # 2) ROI filtering
        if self.roi_enabled:
            x = xyz[:, 0]
            y = xyz[:, 1]
            z = xyz[:, 2]
            mask_roi = (
                (x >= self.roi_min_x) & (x <= self.roi_max_x) &
                (y >= self.roi_min_y) & (y <= self.roi_max_y) &
                (z >= self.roi_min_z) & (z <= self.roi_max_z)
            )
            xyz = xyz[mask_roi]
            pts = pts[mask_roi]

        if xyz.shape[0] == 0:
            return

        # Optionally publish debug filtered cloud
        if self.debug_filtered and self.debug_filtered_pub is not None:
            debug_msg = self.build_cloud(msg.header, pts)
            self.debug_filtered_pub.publish(debug_msg)

        # 3) Clustering
        labels = self.perform_clustering(xyz)
        if labels is None:
            return

        # labels: array of shape (N,), with cluster indices or -1 for noise
        unique_labels = [lab for lab in np.unique(labels) if lab != -1]

        if len(unique_labels) == 0:
            return

        # 4) Build fused cluster cloud with intensity = cluster_id
        cluster_pts = []
        for idx, lab in enumerate(labels):
            if lab == -1:
                continue  # noise
            x, y, z, _ = pts[idx]
            cluster_pts.append((x, y, z, float(lab)))  # store cluster id in intensity

        if cluster_pts:
            cluster_cloud = self.build_cloud(msg.header, np.array(cluster_pts, dtype=np.float32))
            self.cluster_pub.publish(cluster_cloud)

        # 5) Build bounding boxes
        if self.bb_enabled and self.bb_pub is not None:
            markers = self.build_bounding_boxes(msg.header, xyz, labels, unique_labels)
            self.bb_pub.publish(markers)

    # ─────────────────────────────
    # Clustering methods
    # ─────────────────────────────
    def perform_clustering(self, xyz: np.ndarray) -> np.ndarray:
        """
        Perform clustering using DBSCAN backend.
        We interpret 'euclidean' as DBSCAN with euclidean distance.
        """
        if xyz.shape[0] == 0:
            return None

        if self.cluster_method.lower() in ['euclidean', 'dbscan']:
            if self.cluster_method.lower() == 'euclidean':
                eps = self.euclidean_tol
                min_samples = max(self.euclidean_min_size, 2)  # DBSCAN requires >=2
            else:
                eps = self.dbscan_eps
                min_samples = max(self.dbscan_min_points, 2)

            db = DBSCAN(eps=eps, min_samples=min_samples, metric='euclidean')
            labels = db.fit_predict(xyz)

            # Optionally filter clusters by size manually (max size)
            unique_labels = [lab for lab in np.unique(labels) if lab != -1]
            for lab in unique_labels:
                indices = np.where(labels == lab)[0]
                size = len(indices)
                if size < self.euclidean_min_size or size > self.euclidean_max_size:
                    # mark those points as noise
                    labels[indices] = -1

            return labels

        else:
            self.get_logger().warn(f"Unknown clustering method '{self.cluster_method}', skipping.")
            return None

    # ─────────────────────────────
    # Helper: build PointCloud2
    # ─────────────────────────────
    def build_cloud(self, header, pts: np.ndarray) -> PointCloud2:
        """
        Build PointCloud2 from Nx4 array (x, y, z, intensity).
        """
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        points_list = [tuple(p) for p in pts]
        cloud = pc2.create_cloud(header, fields, points_list)
        cloud.header.frame_id = self.frame_id
        return cloud

    # ─────────────────────────────
    # Helper: build bounding boxes
    # ─────────────────────────────
    def build_bounding_boxes(self, header, xyz: np.ndarray,
                             labels: np.ndarray,
                             unique_labels: List[int]) -> MarkerArray:
        """
        Build MarkerArray with one cube Marker per cluster.
        """
        marker_array = MarkerArray()
        marker_id = 0

        for lab in unique_labels:
            indices = np.where(labels == lab)[0]
            if len(indices) == 0:
                continue

            cluster_xyz = xyz[indices]

            min_x = float(np.min(cluster_xyz[:, 0]))
            max_x = float(np.max(cluster_xyz[:, 0]))
            min_y = float(np.min(cluster_xyz[:, 1]))
            max_y = float(np.max(cluster_xyz[:, 1]))
            min_z = float(np.min(cluster_xyz[:, 2]))
            max_z = float(np.max(cluster_xyz[:, 2]))

            # Center of the box
            cx = (min_x + max_x) / 2.0
            cy = (min_y + max_y) / 2.0
            cz = (min_z + max_z) / 2.0

            sx = max_x - min_x
            sy = max_y - min_y
            sz = max_z - min_z

            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.frame_id
            marker.ns = "lidar_clusters"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = cz
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = max(sx, 0.1)
            marker.scale.y = max(sy, 0.1)
            marker.scale.z = max(sz, 0.1)

            # Simple color mapping based on cluster id
            color = self.color_from_id(lab)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.7

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            marker_array.markers.append(marker)
            marker_id += 1

        return marker_array

    def color_from_id(self, cid: int) -> Tuple[float, float, float]:
        """
        Simple deterministic color based on cluster id.
        """
        np.random.seed(cid + 42)
        r = float(np.random.rand())
        g = float(np.random.rand())
        b = float(np.random.rand())
        return r, g, b


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
