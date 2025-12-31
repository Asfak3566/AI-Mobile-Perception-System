import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

class ManualCalibrator(Node):
    def __init__(self):
        super().__init__('manual_calibrator')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Helper to create slider limits for the GUI
        def make_range(min_val, max_val, step):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=float(min_val), to_value=float(max_val), step=float(step))
            ])

        # Declare parameters (These become sliders in rqt_reconfigure)
        # Adjust the default values (0.0) to your rough measurements if you have them
        self.declare_parameter('x', 0.0, make_range(-5.0, 5.0, 0.01))
        self.declare_parameter('y', 0.0, make_range(-5.0, 5.0, 0.01))
        self.declare_parameter('z', 0.0, make_range(-2.0, 2.0, 0.01))
        self.declare_parameter('roll', -1.57, make_range(-3.14, 3.14, 0.01))
        self.declare_parameter('pitch', 0.0, make_range(-3.14, 3.14, 0.01))
        self.declare_parameter('yaw', 0.0, make_range(-3.14, 3.14, 0.01))
        
        # Frame IDs
        self.declare_parameter('parent_frame', 'rslidar')
        self.declare_parameter('child_frame', 'camera_optical_frame')

        # Timer to publish the transform continuously
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback) # 10Hz
        self.get_logger().info("Manual Calibrator Started. Open rqt_reconfigure to adjust sliders!")

    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('parent_frame').value
        t.child_frame_id = self.get_parameter('child_frame').value

        # 1. Read Translation Sliders
        t.transform.translation.x = self.get_parameter('x').value
        t.transform.translation.y = self.get_parameter('y').value
        t.transform.translation.z = self.get_parameter('z').value

        # 2. Read Rotation Sliders (Euler Angles)
        r = self.get_parameter('roll').value
        p = self.get_parameter('pitch').value
        y = self.get_parameter('yaw').value
        
        # 3. Convert Euler to Quaternion
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        t.transform.rotation.w = cy * cp * cr + sy * sp * sr
        t.transform.rotation.x = cy * cp * sr - sy * sp * cr
        t.transform.rotation.y = sy * cp * sr + cy * sp * cr
        t.transform.rotation.z = sy * cp * cr - cy * sp * sr

        # 4. Publish
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ManualCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
