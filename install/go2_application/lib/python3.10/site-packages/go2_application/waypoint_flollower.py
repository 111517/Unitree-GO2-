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
    goal_pose.pose.position.x = 2.0  #
    goal_pose.pose.position.y = 1.0  #
    goal_pose.pose.position.z = 0.0  #
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    goal_poses.append(goal_pose)

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = nav.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0  #
    goal_pose1.pose.position.y = 1.0  #
    goal_pose1.pose.position.z = 0.0  #
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 0.0
    goal_pose1.pose.orientation.z = 0.0
    goal_pose1.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = nav.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.0  #
    goal_pose2.pose.position.y = 0.0  #
    goal_pose2.pose.position.z = 0.0  #
    goal_pose2.pose.orientation.x = 0.0
    goal_pose2.pose.orientation.y = 0.0
    goal_pose2.pose.orientation.z = 0.0
    goal_pose2.pose.orientation.w = 1.0  # 设置初始姿态为无旋转
    goal_poses.append(goal_pose2)
    
    nav.followWaypoints(goal_poses)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f'导航进度: {feedback.current_waypoint}/{feedback.total_waypoints}')
    result = nav.getResult()
    nav.get_logger().info(f"导航结果: {result}")
    # rclpy.spin(nav)
    # nav.destroy_node()
    # rclpy.shutdown()

    