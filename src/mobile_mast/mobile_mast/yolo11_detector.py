import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from builtin_interfaces.msg import Time

import cv2
import numpy as np
from ultralytics import YOLO

class Yolo11DetectorNode(Node):
    def __init__(self):
        super().__init__('yolo11_detector')

        # Declare parameters
        self.declare_parameter('weights', 'yolo11n.pt')
        self.declare_parameter('input_topic', 'image_raw/compressed')
        self.declare_parameter('conf_threshold', 0.35)

        self.weights = self.get_parameter('weights').get_parameter_value().string_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value

        # Log startup
        self.get_logger().info(f"[YOLO11] Loading model from: {self.weights}")
        
        try:
            # Check if we should export to TensorRT
            if self.weights.endswith('.pt'):
                # Check if CUDA is available for TensorRT export
                import torch
                if torch.cuda.is_available():
                    self.get_logger().info(f"[YOLO11] CUDA available, attempting export to TensorRT engine...")
                    # Load the model solely for export
                    model = YOLO(self.weights)
                    # Export to engine (TensorRT)
                    exported_path = model.export(format='engine')
                    self.get_logger().info(f"[YOLO11] Export success: {exported_path}")
                    # Update weights path to the new engine file
                    self.weights = str(exported_path)
                else:
                    self.get_logger().warn(f"[YOLO11] CUDA not available, skipping TensorRT export. Using .pt file directly.")
                    self.get_logger().warn(f"[YOLO11] For TensorRT optimization, install PyTorch with CUDA support.")

            # Load the model (now .engine or original if not .pt)
            self.model = YOLO(self.weights, task='detect')
            self.get_logger().info(f"[YOLO11] Model loaded successfully from {self.weights}")
        except Exception as e:
            self.get_logger().error(f"[YOLO11] Failed to load/export model: {e}")
            raise e

        # Subscriber: input images (CompressedImage or Image)
        # We'll support CompressedImage by default as per launch file, but check topic type if needed.
        # For simplicity, assuming CompressedImage based on launch file default 'image_raw/compressed'
        if 'compressed' in self.input_topic:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.input_topic,
                self.compressed_image_callback,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.input_topic,
                self.image_callback,
                10
            )

        # Publisher: 2D detections
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            'detections_2d',
            10
        )

        self.get_logger().info(f"[YOLO11] Detector node started. Subscribing to {self.input_topic}")

    def compressed_image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn("Failed to decode compressed image")
            return
        self.process_image(cv_image, msg.header)

    def image_callback(self, msg: Image):
        # Manual conversion for sensor_msgs/Image to OpenCV
        # Assuming bgr8 or rgb8
        dtype = np.uint8
        n_channels = 3
        if msg.encoding == 'mono8':
            n_channels = 1
        
        img_buf = np.frombuffer(msg.data, dtype=dtype)
        try:
            cv_image = img_buf.reshape((msg.height, msg.width, n_channels))
            # If rgb8, convert to bgr8 for OpenCV
            if msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except ValueError as e:
            self.get_logger().error(f"Failed to reshape image: {e}")
            return

        self.process_image(cv_image, msg.header)

    def process_image(self, cv_image, header):
        # Run detection
        # stream=True is efficient for video streams, but here we process one by one
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        det_array = Detection2DArray()
        det_array.header = header

        if results and len(results) > 0:
            result = results[0] # We only passed one image
            
            for box in result.boxes:
                # box.xywh is center_x, center_y, width, height
                xywh = box.xywh[0].cpu().numpy()
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                
                # Get class name if available
                class_name = self.model.names[cls_id] if self.model.names else str(cls_id)

                det = Detection2D()
                det.header = header
                
                det.bbox.center.x = float(xywh[0])
                det.bbox.center.y = float(xywh[1])
                det.bbox.size_x = float(xywh[2])
                det.bbox.size_y = float(xywh[3])

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = class_name
                hyp.hypothesis.score = conf
                
                det.results.append(hyp)
                det_array.detections.append(det)

        self.detections_pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    node = Yolo11DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
