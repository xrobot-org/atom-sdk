#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <chrono>  // 高精度时间处理
#include <ctime>   // 标准时间处理函数
#include <fstream> // 文件输出
#include <iomanip> // 设置浮点精度与时间格式
#include <sstream> // 字符串流

class SubscriberNode : public rclcpp::Node {
public:
  SubscriberNode() : Node("subscriber_node") {
    // 打开CSV文件（覆盖旧内容）
    csv_file_.open("imu_data.csv", std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开 imu_data.csv 文件！");
    } else {
      // 写入表头
      csv_file_ << "Time,ax,ay,az,gx,gy,gz,q0,q1,q2,q3\n";
      RCLCPP_INFO(this->get_logger(), "已打开 imu_data.csv，并写入表头");
    }

    // 创建订阅者
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "atom/imu_data", rclcpp::SensorDataQoS(),
        std::bind(&SubscriberNode::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "IMU 订阅节点已启动，等待数据中...");
  }

  ~SubscriberNode() override {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV 文件已关闭");
    }
  }

  std::string FormatStampToString(const builtin_interfaces::msg::Time &stamp) {
    std::chrono::system_clock::time_point t =
        std::chrono::system_clock::time_point{} +
        std::chrono::seconds(stamp.sec) +
        std::chrono::nanoseconds(stamp.nanosec);

    auto t_us = std::chrono::time_point_cast<std::chrono::microseconds>(t);
    std::time_t tt = std::chrono::system_clock::to_time_t(t_us);
    auto us_part = t_us.time_since_epoch().count() % 1000000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&tt), "%Y-%m-%d %H:%M:%S") << "."
       << std::setw(6) << std::setfill('0') << us_part;
    return ss.str();
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 格式化时间戳（到微秒）
    std::string timestamp_str = FormatStampToString(msg->header.stamp);

    // 输出日志
    RCLCPP_INFO(
        this->get_logger(),
        "[%s] ax: %+6f, ay: %+6f, az: %+6f | gx: %+6f, gy: %+6f, gz: %+6f | "
        "q0: %+6f, q1: %+6f, q2: %+6f, q3: %+6f",
        timestamp_str.c_str(), msg->linear_acceleration.x,
        msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z, msg->orientation.w, msg->orientation.x,
        msg->orientation.y, msg->orientation.z);

    // 写入CSV
    if (csv_file_.is_open()) {
      csv_file_ << timestamp_str.c_str() << "," << msg->linear_acceleration.x
                << "," << msg->linear_acceleration.y << ","
                << msg->linear_acceleration.z << "," << msg->angular_velocity.x
                << "," << msg->angular_velocity.y << ","
                << msg->angular_velocity.z << "," << msg->orientation.w << ","
                << msg->orientation.x << "," << msg->orientation.y << ","
                << msg->orientation.z << "\n";
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  std::ofstream csv_file_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}