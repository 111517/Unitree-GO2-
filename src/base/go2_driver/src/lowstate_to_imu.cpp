#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class LowStateToImuNode : public rclcpp::Node
{
public:
    LowStateToImuNode() : Node("lowstate_to_imu_node")
    {
        // 参数
        this->declare_parameter<std::string>("imu_frame", "imu");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<bool>("cartographer_2d_mode", true); // true: 2D, false: 3D

        this->get_parameter("imu_frame", imu_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("cartographer_2d_mode", carto_2d_mode_);

        // 1. 订阅 LowState 消息
        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lf/lowstate", 100,
            std::bind(&LowStateToImuNode::lowstate_callback, this, std::placeholders::_1));

        // 2. 发布标准 IMU 话题
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 100);

        // 3. 创建 TF 发布器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "LowState → IMU 节点已启动");
    }

private:
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string imu_frame_;
    std::string base_frame_;
    bool carto_2d_mode_;

    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr lowstate_msg)
    {
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

        // 时间戳
        imu_msg->header.stamp = this->now(); // 如果 lowstate_msg 有时间戳可替换
        imu_msg->header.frame_id = imu_frame_;

        // 四元数转换 Unitree [w,x,y,z] -> ROS [x,y,z,w]
        const auto& quat = lowstate_msg->imu_state.quaternion;
        imu_msg->orientation.x = quat[1];
        imu_msg->orientation.y = quat[2];
        imu_msg->orientation.z = quat[3];
        imu_msg->orientation.w = quat[0];

        // 角速度
        imu_msg->angular_velocity.x = lowstate_msg->imu_state.gyroscope[0];
        imu_msg->angular_velocity.y = lowstate_msg->imu_state.gyroscope[1];
        imu_msg->angular_velocity.z = lowstate_msg->imu_state.gyroscope[2];

        // 加速度
        imu_msg->linear_acceleration.x = lowstate_msg->imu_state.accelerometer[0];
        imu_msg->linear_acceleration.y = lowstate_msg->imu_state.accelerometer[1];
        imu_msg->linear_acceleration.z = lowstate_msg->imu_state.accelerometer[2];

        // 协方差设置
        // 角速度协方差
        imu_msg->angular_velocity_covariance[0] = 0.0002;
        imu_msg->angular_velocity_covariance[4] = 0.0002;
        imu_msg->angular_velocity_covariance[8] = 0.0002;

        // 线性加速度协方差
        imu_msg->linear_acceleration_covariance[0] = 0.02;
        imu_msg->linear_acceleration_covariance[4] = 0.02;
        imu_msg->linear_acceleration_covariance[8] = 0.02;

        // 方向协方差根据 Cartographer 2D/3D 自动设置
        if (carto_2d_mode_) {
            // 2D SLAM: 只信任 yaw
            imu_msg->orientation_covariance[0] = 9999;   // roll 不可信
            imu_msg->orientation_covariance[4] = 9999;   // pitch 不可信
            imu_msg->orientation_covariance[8] = 0.0001; // yaw 很可信
        } else {
            // 3D SLAM: orientation 全可信
            imu_msg->orientation_covariance[0] = 0.001;
            imu_msg->orientation_covariance[4] = 0.001;
            imu_msg->orientation_covariance[8] = 0.001;
        }

        imu_pub_->publish(std::move(imu_msg));

        // 发布 TF
        publish_tf_transform();
    }

    void publish_tf_transform()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = base_frame_;
        tf.child_frame_id = imu_frame_;

        // IMU 相对 base_link 的位置，从 URDF 获取
        tf.transform.translation.x = -0.02557;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.04232;

        // 使用单位四元数（如需要可以改成 imu orientation）
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowStateToImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
