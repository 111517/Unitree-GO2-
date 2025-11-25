from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0  # 设置初始位置的x坐标
    goal_pose.pose.position.y = 1.0  # 设置初始位置的y
    goal_pose.pose.position.z = 0.0  # 设置初始位置的z坐标
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f"导航进度: {feedback.current_waypoint}/{feedback.total_waypoints}")
        nav.get_logger().info(f"距离目标剩余距离: {feedback.distance_remaining:.2f} 米")
    result = nav.getResult()
    nav.get_logger().info(f"导航结果: {result}")
    # rclpy.spin(nav)
    # nav.destroy_node()
    # rclpy.shutdown()

    