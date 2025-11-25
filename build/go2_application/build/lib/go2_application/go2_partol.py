from geometry_msgs.msg import PoseStamped,Pose
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion,quaternion_from_euler
import math

class PartolNode(BasicNavigator):
    def __init__(self, node_name='go2_partol'):
        super().__init__(node_name)
        
        self.declare_parameter('initial_point',[0.0,0.0,0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose_by_xyyaw(self,x,y,yaw):
        '''
        return  PoseStamped
        '''
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x=x
        pose.pose.position.y=y
        quat = quaternion_from_euler(0.0,0.0,yaw)
        pose.pose.orientation.x=quat[0]
        pose.pose.orientation.y=quat[1]
        pose.pose.orientation.z=quat[2]
        pose.pose.orientation.w=quat[3]
        return pose

    def init_go2_pose(self):
        '''
        initial go2 pose
        '''
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0],self.initial_point_[1],self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

    def get_target_points(self):
        '''
        get target points
        '''
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f"target point {index}: x={x}, y={y}, yaw={yaw}")
        return points

    def nav_to_pose(self,target_pose):
        '''
        navigation to points
        '''
        self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f"导航进度: {feedback.current_waypoint}/{feedback.total_waypoints}")
            self.get_logger().info(f"距离目标剩余距离: {feedback.distance_remaining:.2f} 米")
        result = self.getResult()
        self.get_logger().info(f"导航结果: {result}")

    def get_current_pose(self):
        '''
        get current pose
        '''
        while rclpy.ok():
            try:
                result = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0.8),rclpy.time.Duration(seconds=1.0))
                transform = result.transform
                self.get_logger().info(f"Translation: x={transform.translation.x}, y={transform.translation.y}, z={transform.translation.z}")
                # self.get_logger().info(f"Rotation: x={transform.rotation.x}, y={transform.rotation.y}, z={transform.rotation.z}, w={transform.rotation.w}")
                # rotation_euler = euler_from_quaternion([
                #     transform.rotation.x,
                #     transform.rotation.y,
                #     transform.rotation.z,
                #     transform.rotation.w
                # ])
                # self.get_logger().info(f"Euler angles: roll={math.degrees(rotation_euler[0])}, pitch={math.degrees(rotation_euler[1])}, yaw={math.degrees(rotation_euler[2])}")
                return transform
            except Exception as e:
                self.get_logger().error(f"Could not transform: {e}")

def main():
    rclpy.init()
    partol = PartolNode()
    partol.init_go2_pose()
    while rclpy.ok():
        points = partol.get_target_points()
        for point in points:
            target_pose = partol.get_pose_by_xyyaw(point[0],point[1],point[2])
            partol.nav_to_pose(target_pose)
    partol.destroy_node()
    rclpy.shutdown()