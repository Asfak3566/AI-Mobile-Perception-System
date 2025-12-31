# file: mobile_mast/rslidar_preprocess.py

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class RslidarPreprocessNode(Node):
    """
    LiDAR preprocessing node for RoboSense data.

    Steps:
      1. Subscribes to raw RoboSense point cloud (PointCloud2).
      2. Filters by range [min_range, max_range].
      3. Removes ground points (simple z-based threshold).
      4. Applies ROI box filtering in x/y/z.
      5. Publishes filtered cloud as PointCloud2.

    Default topics:
      input : /mast/orin1/lidar/points
      output: /mast/orin1/lidar/points_filtered
    """

    def __init__(self):
        super().__init__('rslidar_preprocess')

        # ───────────────── Parameters ─────────────────
        # Topics
        self.declare_parameter('input_topic', '/mast/orin1/lidar/points')
        self.declare_parameter('output_topic', '/mast/orin1/lidar/points_filtered')

        # Range filter [m]
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 200.0)

        # Ground removal
        # Simple approach: remove points with z between ground_z_min and ground_z_max
        # (Assuming LiDAR is mounted such that ground is around z=0 in LiDAR frame)
        self.declare_parameter('enable_ground_removal', True)
        self.declare_parameter('ground_z_min', -0.3)
        self.declare_parameter('ground_z_max', 0.3)

        # ROI filter (box in LiDAR frame)
        # Keep only points inside [min_x, max_x], [min_y, max_y], [min_z, max_z]
        self.declare_parameter('enable_roi_filter', False)
        self.declare_parameter('roi_min_x', -50.0)
        self.declare_parameter('roi_max_x',  50.0)
        self.declare_parameter('roi_min_y', -50.0)
        self.declare_parameter('roi_max_y',  50.0)
        self.declare_parameter('roi_min_z', -3.0)
        self.declare_parameter('roi_max_z',  5.0)

        # ───────────────── Read parameters ─────────────────
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value

        self.enable_ground_removal = self.get_parameter('enable_ground_removal').get_parameter_value().bool_value
        self.ground_z_min = self.get_parameter('ground_z_min').get_parameter_value().double_value
        self.ground_z_max = self.get_parameter('ground_z_max').get_parameter_value().double_value

        self.enable_roi_filter = self.get_parameter('enable_roi_filter').get_parameter_value().bool_value
        self.roi_min_x = self.get_parameter('roi_min_x').get_parameter_value().double_value
        self.roi_max_x = self.get_parameter('roi_max_x').get_parameter_value().double_value
        self.roi_min_y = self.get_parameter('roi_min_y').get_parameter_value().double_value
        self.roi_max_y = self.get_parameter('roi_max_y').get_parameter_value().double_value
        self.roi_min_z = self.get_parameter('roi_min_z').get_parameter_value().double_value
        self.roi_max_z = self.get_parameter('roi_max_z').get_parameter_value().double_value

        # ───────────────── Logging ─────────────────
        self.get_logger().info(f"[RSLIDAR PREPROCESS] input_topic : {self.input_topic}")
        self.get_logger().info(f"[RSLIDAR PREPROCESS] output_topic: {self.output_topic}")
        self.get_logger().info(f"[RSLIDAR PREPROCESS] range filter: [{self.min_range}, {self.max_range}] m")

        if self.enable_ground_removal:
            self.get_logger().info(
                f"[RSLIDAR PREPROCESS] ground removal: z in [{self.ground_z_min}, {self.ground_z_max}] will be REMOVED"
            )
        else:
            self.get_logger().info("[RSLIDAR PREPROCESS] ground removal: DISABLED")

        if self.enable_roi_filter:
            self.get_logger().info(
                f"[RSLIDAR PREPROCESS] ROI enabled: "
                f"x[{self.roi_min_x}, {self.roi_max_x}], "
                f"y[{self.roi_min_y}, {self.roi_max_y}], "
                f"z[{self.roi_min_z}, {self.roi_max_z}]"
            )
        else:
            self.get_logger().info("[RSLIDAR PREPROCESS] ROI filter: DISABLED")

        # ───────────────── ROS I/O ─────────────────
        qos = QoSProfile(depth=10)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos
        )

    def cloud_callback(self, msg: PointCloud2):
        """
        Callback: filter input cloud and publish filtered result.
        """
        # Read points (x, y, z, intensity)
        points = pc2.read_points_list(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        )

        filtered_points = []

        for p in points:
            x, y, z, intensity = p

            # 1) Range filter
            r = math.sqrt(x * x + y * y + z * z)
            if r < self.min_range or r > self.max_range:
                continue

            # 2) Ground removal
            if self.enable_ground_removal:
                if self.ground_z_min <= z <= self.ground_z_max:
                    # This point is considered ground → skip
                    continue

            # 3) ROI filter
            if self.enable_roi_filter:
                if not (self.roi_min_x <= x <= self.roi_max_x):
                    continue
                if not (self.roi_min_y <= y <= self.roi_max_y):
                    continue
                if not (self.roi_min_z <= z <= self.roi_max_z):
                    continue

            filtered_points.append((x, y, z, intensity))

        if not filtered_points:
            # Nothing to publish
            return

        # Create new PointCloud2
        header = msg.header
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_out = pc2.create_cloud(header, fields, filtered_points)
        self.pub.publish(cloud_out)


def main(args=None):
    rclpy.init(args=args)
    node = RslidarPreprocessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
