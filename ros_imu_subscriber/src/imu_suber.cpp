#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class SubscriberNode : public rclcpp::Node {
public:
  SubscriberNode() : Node("subscriber_node") {
    // 创建订阅者，订阅IMU消息
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "atom/imu_data", rclcpp::SensorDataQoS(),
        std::bind(&SubscriberNode::imu_callback, this, std::placeholders::_1));
  }

private:
  // IMU消息的回调函数
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "ax: %+6f, ay: %+6f, az: %+6f gx: %+6f, gy: %+6f, gz: %+6f q0: "
                "%+6f, q1: %+6f, q2: %+6f, q3: %+6f",
                msg->linear_acceleration.x, msg->linear_acceleration.y,
                msg->linear_acceleration.z, msg->angular_velocity.x,
                msg->angular_velocity.y, msg->angular_velocity.z,
                msg->orientation.w, msg->orientation.x, msg->orientation.y,
                msg->orientation.z);
  }

  // 订阅者成员变量
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}