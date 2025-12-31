import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    """
    Publishes sensor_msgs/CameraInfo from a YAML file at a fixed rate.
    Useful when a camera driver does not publish camera_info.
    """

    def __init__(self):
        super().__init__("camera_info_publisher")

        self.declare_parameter("camera_info_yaml", "")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("publish_rate_hz", 5.0)

        yaml_path = self.get_parameter("camera_info_yaml").get_parameter_value().string_value
        topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        if not yaml_path:
            raise RuntimeError("camera_info_yaml parameter is empty")

        self.cam_info = self._load_camera_info(yaml_path)
        self.pub = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._on_timer)

        self.get_logger().info(f"Publishing CameraInfo on '{topic}' from '{yaml_path}'")

    def _load_camera_info(self, yaml_path: str) -> CameraInfo:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        msg = CameraInfo()
        msg.width = int(data["image_width"])
        msg.height = int(data["image_height"])
        msg.distortion_model = data.get("distortion_model", "plumb_bob")

        # D
        msg.d = [float(x) for x in data["distortion_coefficients"]["data"]]

        # K (3x3)
        msg.k = [float(x) for x in data["camera_matrix"]["data"]]

        # R (3x3)
        msg.r = [float(x) for x in data["rectification_matrix"]["data"]]

        # P (3x4)
        msg.p = [float(x) for x in data["projection_matrix"]["data"]]

        return msg

    def _on_timer(self):
        # Keep timestamps current
        self.cam_info.header.stamp = self.get_clock().now().to_msg()
        self.cam_info.header.frame_id = self.frame_id
        self.pub.publish(self.cam_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

