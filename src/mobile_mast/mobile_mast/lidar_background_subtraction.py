# file: mobile_mast/lidar_background_subtraction.py

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class LidarBackgroundSubtractionNode(Node):
    """
    Background subtraction for LiDAR.

    - Loads a static background point cloud (PCD file, ASCII format).
    - Builds a voxel grid of background points.
    - For each incoming live cloud, removes points that fall into background voxels.
    - Publishes the remaining (foreground) points.

    Parameters:
      input_topic        : live filtered cloud (e.g., /mast/orin1/lidar/points_filtered)
      output_topic       : foreground cloud (e.g., /mast/orin1/lidar/foreground)
      background_pcd_file: path to background.pcd (ASCII PCD)
      voxel_size         : meters, defines how "similar" a point must be to background
    """

    def __init__(self):
        super().__init__('lidar_background_subtraction')

        # Declare parameters
        self.declare_parameter('input_topic', '/mast/orin1/lidar/points_filtered')
        self.declare_parameter('output_topic', '/mast/orin1/lidar/foreground')
        self.declare_parameter('background_pcd_file', '')
        self.declare_parameter('voxel_size', 0.3)  # meters

        # Read parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.background_pcd_file = self.get_parameter('background_pcd_file').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value

        if not self.background_pcd_file:
            self.get_logger().error("Parameter 'background_pcd_file' is empty. Cannot run background subtraction.")
            raise RuntimeError("background_pcd_file not set")

        if not os.path.isfile(self.background_pcd_file):
            self.get_logger().error(f"Background PCD file does not exist: {self.background_pcd_file}")
            raise RuntimeError("background_pcd_file not found")

        self.get_logger().info(f"[BG SUB] input_topic        : {self.input_topic}")
        self.get_logger().info(f"[BG SUB] output_topic       : {self.output_topic}")
        self.get_logger().info(f"[BG SUB] background_pcd_file: {self.background_pcd_file}")
        self.get_logger().info(f"[BG SUB] voxel_size         : {self.voxel_size}")

        # Load background PCD
        self.bg_voxels = self.load_background_voxels(self.background_pcd_file, self.voxel_size)
        self.get_logger().info(f"[BG SUB] Loaded {len(self.bg_voxels)} background voxels.")

        qos = QoSProfile(depth=10)

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        # Publisher
        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos
        )

    # ─────────────────────────────
    # Background loading
    # ─────────────────────────────
    def load_background_voxels(self, pcd_path: str, voxel_size: float):
        """
        Load an ASCII PCD file and build a set of voxel indices representing background.

        We assume:
          - The PCD has FIELDS including 'x', 'y', 'z'
          - DATA ascii
        """

        with open(pcd_path, 'r') as f:
            lines = f.readlines()

        fields = []
        data_start_idx = None

        # Parse header
        for i, line in enumerate(lines):
            line = line.strip()
            if line.startswith('FIELDS'):
                # Example: "FIELDS x y z intensity"
                fields = line.split()[1:]
            if line.startswith('DATA'):
                if 'ascii' not in line:
                    self.get_logger().error(
                        "PCD file DATA is not ascii. Please save background.pcd as ASCII format."
                    )
                    raise RuntimeError("PCD DATA must be ascii")
                data_start_idx = i + 1
                break

        if data_start_idx is None:
            self.get_logger().error("Could not find DATA line in PCD header.")
            raise RuntimeError("Invalid PCD file")

        # Find indices of x, y, z
        try:
            ix = fields.index('x')
            iy = fields.index('y')
            iz = fields.index('z')
        except ValueError:
            self.get_logger().error(f"FIELDS line does not contain x,y,z: {fields}")
            raise

        bg_voxels = set()

        for line in lines[data_start_idx:]:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) <= max(ix, iy, iz):
                continue

            try:
                x = float(parts[ix])
                y = float(parts[iy])
                z = float(parts[iz])
            except ValueError:
                continue

            vx = int(math.floor(x / voxel_size))
            vy = int(math.floor(y / voxel_size))
            vz = int(math.floor(z / voxel_size))
            bg_voxels.add((vx, vy, vz))

        return bg_voxels

    # ─────────────────────────────
    # Live cloud callback
    # ─────────────────────────────
    def cloud_callback(self, msg: PointCloud2):
        # Read points (x, y, z, intensity if available)
        points = pc2.read_points_list(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        )
        if not points:
            return

        foreground_points = []
        voxel_size = self.voxel_size

        for p in points:
            x, y, z, intensity = p
            vx = int(math.floor(x / voxel_size))
            vy = int(math.floor(y / voxel_size))
            vz = int(math.floor(z / voxel_size))

            if (vx, vy, vz) in self.bg_voxels:
                # This point belongs to background → skip
                continue

            # Keep foreground point
            foreground_points.append((x, y, z, intensity))

        if not foreground_points:
            # No foreground points detected
            return

        # Build new PointCloud2 message
        header = msg.header
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_out = pc2.create_cloud(header, fields, foreground_points)
        self.pub.publish(cloud_out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBackgroundSubtractionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
