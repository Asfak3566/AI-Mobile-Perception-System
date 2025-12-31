import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

# NVIDIA VPI is now a mandatory dependency
try:
    import vpi
except ImportError:
    raise ImportError("NVIDIA VPI Python bindings not found. This node requires VPI and NVIDIA hardware.")

class VpiUndistortNode(Node):
    """
    Subscribes to Image + CameraInfo and publishes undistorted Image using NVIDIA VPI.
    This implementation is platform-specific (NVIDIA Jetson/Orin)
    """

    def __init__(self):
        super().__init__("vpi_undistort_node")

        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("output_topic", "image_rect")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("backend", "cuda") # 'cuda'

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        caminfo_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.output_frame_id = self.get_parameter("output_frame_id").get_parameter_value().string_value
        backend_str = self.get_parameter("backend").get_parameter_value().string_value

        if backend_str.lower() == "vic":
            self.vpi_backend = vpi.Backend.VIC
        else:
            self.vpi_backend = vpi.Backend.CUDA

        self.pub = self.create_publisher(Image, output_topic, 10)

        self.sub_img = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.sub_info = self.create_subscription(CameraInfo, caminfo_topic, self._on_info, 10)

        self.have_map = False
        self.remap_obj = None
        self.last_info = None
        self.stream = vpi.Stream()

        self.get_logger().info(
            f"VPI Undistort Node started using {backend_str.upper()} backend. "
            f"Subscribing: {image_topic}, {caminfo_topic} → Publishing: {output_topic}"
        )

    def _on_info(self, msg: CameraInfo):
        # Update calibration if it changes (rare in practice but handled)
        if self.last_info is None or \
           self.last_info.k != msg.k or \
           self.last_info.d != msg.d or \
           self.last_info.width != msg.width or \
           self.last_info.height != msg.height:
            self.last_info = msg
            self.have_map = False # Force rebuild
            self._build_maps_if_possible()

    def _build_maps_if_possible(self):
        if self.last_info is None or self.have_map:
            return

        w, h = int(self.last_info.width), int(self.last_info.height)
        
        # Camera intrinsics K
        K = np.array(self.last_info.k).reshape(3, 3)
        
        # Distortion coefficients D
        # ROS plumb_bob: [k1, k2, p1, p2, k3]
        # VPI polynomial model expects: [k1, k2, p1, p2, k3, k4, k5, k6]
        d_ros = list(self.last_info.d)
        while len(d_ros) < 8:
            d_ros.append(0.0)
        
        # VPI expects specific model. 
        # For plumb_bob, we use Polynomial Lens Distortion Model.
        # Note: VPI's polynomial model maps from undistorted to distorted (for remapping).
        
        try:
            # Create the warp map
            warp = vpi.WarpMap(vpi.WarpMap.Type.REMAP, (w, h))
            
            # Intrinsic matrix for VPI
            intrinsic = [
                [K[0,0], K[0,1], K[0,2]],
                [K[1,0], K[1,1], K[1,2]],
                [0,      0,      1     ]
            ]
            
            # For identity rectification
            extrinsic = [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0]
            ]

            # Generate the polynomial lens distortion map
            # This fills the WarpMap to perform undistortion
            warp.generate_polynomial_lens_distortion(intrinsic, extrinsic, intrinsic, d_ros)
            
            # Create persistent Remap object for performance
            self.remap_obj = vpi.Remap(warp, interp=vpi.Interp.LINEAR, border=vpi.Border.ZERO)
            
            self.have_map = True
            self.get_logger().info(f"VPI Undistortion maps built for {w}x{h} image.")
        except Exception as e:
            self.get_logger().error(f"Failed to build VPI maps: {e}")

    def _on_image(self, msg: Image):
        if self.last_info is None:
            self.pub.publish(msg)
            return

        if not self.have_map:
            self._build_maps_if_possible()
            if not self.have_map:
                self.pub.publish(msg)
                return

        # Manual conversion: ROS Image -> Numpy
        # Assuming BGR8 encoding as per previous implementation
        if msg.encoding != "bgr8":
            self.get_logger().warn(f"Unsupported encoding {msg.encoding}, expected bgr8. Attempting passthrough.", once=True)
            self.pub.publish(msg)
            return

        try:
            # Convert ROS data to numpy array
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

            with self.stream:
                # Wrap numpy array in VPI Image
                vpi_img = vpi.asimage(frame)
                
                # Execute remap on selected backend (CUDA/VIC)
                undist_vpi = self.remap_obj(vpi_img, backend=self.vpi_backend)
                
                # Get back to cpu/numpy
                undist_cpu = undist_vpi.cpu()

            # Create output ROS image
            out = Image()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.output_frame_id if self.output_frame_id else msg.header.frame_id
            out.height = undist_cpu.shape[0]
            out.width = undist_cpu.shape[1]
            out.encoding = "bgr8"
            out.is_bigendian = msg.is_bigendian
            out.step = undist_cpu.shape[1] * 3
            out.data = undist_cpu.tobytes()
            
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(f"VPI processing failed: {e}")
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VpiUndistortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

