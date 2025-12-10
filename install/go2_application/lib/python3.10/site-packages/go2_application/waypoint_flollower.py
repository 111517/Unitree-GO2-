from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    goal_poses=[]

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.6457520723342896 #
    goal_pose.pose.position.y = -0.009389510378241539  #
    goal_pose.pose.position.z = -0.005340576171875  #
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    goal_poses.append(goal_pose)

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = nav.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.7333511114120483 #
    goal_pose1.pose.position.y = -2.0111658573150635  #
    goal_pose1.pose.position.z = -0.001434326171875  #
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 0.0
    goal_pose1.pose.orientation.z = 0.0
    goal_pose1.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = nav.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.6945875883102417  #
    goal_pose2.pose.position.y = -2.0049068927764893  #
    goal_pose2.pose.position.z = -0.005340576171875  #
    goal_pose2.pose.orientation.x = 0.0
    goal_pose2.pose.orientation.y = 0.0
    goal_pose2.pose.orientation.z = 0.0
    goal_pose2.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    goal_poses.append(goal_pose2)
    
    nav.followWaypoints(goal_poses)
    result = nav.getResult()
    nav.get_logger().info(f"导航结果: {result}")
    # rclpy.spin(nav)
    # nav.destroy_node()
    # rclpy.shutdown()

    