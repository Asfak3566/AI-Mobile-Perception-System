# file: mobile_mast/tracks_to_markers_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class TracksToMarkersNode(Node):
    """
    Convert Detection3DArray (tracks_3d) into MarkerArray for RViz visualization.

    Subscribes:
      - Detection3DArray on tracks_topic (default: /mast/global/tracks_3d)

    Publishes:
      - MarkerArray on markers_topic (default: /mast/global/tracks_markers)
    """

    def __init__(self):
        super().__init__('tracks_to_markers_node')

        self.declare_parameter('tracks_topic', '/mast/global/tracks_3d')
        self.declare_parameter('markers_topic', '/mast/global/tracks_markers')

        self.tracks_topic = self.get_parameter('tracks_topic').get_parameter_value().string_value
        self.markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value

        self.get_logger().info(f"[TRACKS2MARKERS] Subscribing to: {self.tracks_topic}")
        self.get_logger().info(f"[TRACKS2MARKERS] Publishing   to: {self.markers_topic}")

        qos = QoSProfile(depth=10)

        self.sub = self.create_subscription(
            Detection3DArray,
            self.tracks_topic,
            self.callback,
            qos,
        )

        self.pub = self.create_publisher(
            MarkerArray,
            self.markers_topic,
            qos,
        )

    def callback(self, msg: Detection3DArray):
        markers = MarkerArray()

        stamp = msg.header.stamp
        frame_id = msg.header.frame_id or "rslidar"

        # We'll reuse marker IDs every callback (overwrite old markers)
        for i, det in enumerate(msg.detections):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id

            m.ns = "global_tracks"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD

            # Position = bbox center
            m.pose = det.bbox.center

            # Size from bbox.size (fallback if zero)
            sx = det.bbox.size.x if det.bbox.size.x > 0.0 else 1.5
            sy = det.bbox.size.y if det.bbox.size.y > 0.0 else 1.5
            sz = det.bbox.size.z if det.bbox.size.z > 0.0 else 1.5
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz

            # Color: semi-transparent green
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.7

            # Lifetime: short, so old markers disappear automatically
            m.lifetime = Duration(seconds=0.2).to_msg()

            # Optional: put track id / class into text
            if det.results and det.results[0].hypothesis.class_id:
                m.text = det.results[0].hypothesis.class_id

            markers.markers.append(m)

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TracksToMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
