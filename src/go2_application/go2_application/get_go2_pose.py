import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_go2_pose)

    def get_go2_pose(self):
        try:
            result = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0.8),rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f"Translation: x={transform.translation.x}, y={transform.translation.y}, z={transform.translation.z}")
            self.get_logger().info(f"Rotation: x={transform.rotation.x}, y={transform.rotation.y}, z={transform.rotation.z}, w={transform.rotation.w}")
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(f"Euler angles: roll={math.degrees(rotation_euler[0])}, pitch={math.degrees(rotation_euler[1])}, yaw={math.degrees(rotation_euler[2])}")
        except Exception as e:
            self.get_logger().error(f"Could not transform: {e}")

    def main(args=None):
        rclpy.init(args=args)
        tf_broadcaster = TFBroadcaster()
        rclpy.spin(tf_broadcaster)
        tf_broadcaster.destroy_node()
        rclpy.shutdown()