from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    rclpy.init()
    nav = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.0  # 设置初始位置的x坐标
    initial_pose.pose.position.y = 1.0  # 设置初始位置的y
    initial_pose.pose.position.z = 0.0  # 设置初始位置的z坐标
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()
    rclpy.spin(nav)
    nav.destroy_node()
    rclpy.shutdown()

    