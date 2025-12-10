#!/usr/bin/env python3

# ================= NUMPY 2.x COMPATIBILITY PATCH ==================
import numpy as _np
if not hasattr(_np, "float"):
    _np.float = float
if not hasattr(_np, "maximum_sctype"):
    def _maximum_sctype_dummy(dtype_like):
        return _np.float64
    _np.maximum_sctype = _maximum_sctype_dummy
# ==================================================================

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import rclpy.duration
import os
import time
import cv2

from go2_camera.go2_video import get_go2_image
from go2_camera.yolo_detector import YoloDetector


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='go2_patrol'):
        super().__init__(node_name)

        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [
            1.972, -0.023, 0.018,
            1.963, -1.839, -1.555,
            0.550, -1.949, -3.130
        ])
        self.declare_parameter('photo_save_path', "/home/ayi/go2_patrol_photos/")

        self.initial_point_ = self.get_parameter('initial_point').get_parameter_value().double_array_value
        self.target_points_ = self.get_parameter('target_points').get_parameter_value().double_array_value
        self.photo_save_path_ = self.get_parameter('photo_save_path').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO init
        try:
            self.yolo = YoloDetector("/home/ayi/unitree_go2_ws/src/go2_camera/yolov8n.pt")
            self.get_logger().info("YOLO检测器初始化成功")
        except Exception as e:
            self.get_logger().error(f"YOLO初始化失败: {e}")
            raise RuntimeError("YOLO init failed")

        os.makedirs(self.photo_save_path_, exist_ok=True)

    # =========== 生成 Pose ============
    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    # =========== 解析航点 ============
    def get_target_points(self):
        points = []
        if len(self.target_points_) % 3 != 0:
            self.get_logger().error("目标航点数量错误")
            return points

        for i in range(int(len(self.target_points_) / 3)):
            x = self.target_points_[i * 3]
            y = self.target_points_[i * 3 + 1]
            yaw = self.target_points_[i * 3 + 2]
            points.append([x, y, yaw])

        return points

    # =========== 初始化位姿 ============
    def init_go2_pose(self):
        init = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init)
        self.waitUntilNav2Active()

    # =========== 导航 ============
    def nav_to_pose(self, pose):
        self.goToPose(pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"导航中 | 剩余距离：{feedback.distance_remaining:.2f}m")

        result = self.getResult()
        return result == TaskResult.SUCCEEDED

    # =========== 拍照 + YOLO ============
    def take_photo(self, loop_count, point_index):
        self.get_logger().info(f"执行拍照：第{loop_count}轮，第{point_index}个航点")

        frame = None
        for retry in range(3):
            frame = get_go2_image()
            if frame is not None:
                break
            self.get_logger().warn(f"图像获取失败，重试 {retry+1}/3")
            time.sleep(0.1)

        if frame is None:
            self.get_logger().error("拍照失败，无图像")
            return

        try:
            _, annotated, dets = self.yolo.detect(frame)
            self.get_logger().info(f"YOLO检测结果：{dets}")
        except:
            annotated = frame

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_loop_{loop_count}_point_{point_index}.jpg"
        save_path = os.path.join(self.photo_save_path_, filename)

        cv2.imwrite(save_path, annotated)
        self.get_logger().info(f"照片保存：{save_path}")

    # =========== 获取当前位姿 ============
    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            rot = tf.transform.rotation
            yaw = math.degrees(euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2])
            self.get_logger().info(f"当前位姿: yaw={yaw:.1f}°")
        except:
            pass


def main():
    rclpy.init()

    patrol_node = PatrolNode()
    patrol_node.init_go2_pose()
    points = patrol_node.get_target_points()

    loop_count = 0

    try:
        while rclpy.ok():
            loop_count += 1
            patrol_node.get_logger().info(f"===== 第 {loop_count} 轮巡航开始 =====")

            for i, p in enumerate(points):
                pose = patrol_node.get_pose_by_xyyaw(p[0], p[1], p[2])

                if patrol_node.nav_to_pose(pose):
                    patrol_node.take_photo(loop_count, i+1)
                    patrol_node.get_current_pose()

                rclpy.spin_once(patrol_node, timeout_sec=0.5)

            patrol_node.get_logger().info(f"===== 第 {loop_count} 轮巡航结束 =====\n")

            time.sleep(1.0)

    except KeyboardInterrupt:
        patrol_node.get_logger().warn("用户终止")

    finally:
        cv2.destroyAllWindows()
        patrol_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
