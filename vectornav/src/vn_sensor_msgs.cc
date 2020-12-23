#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vectornav_msgs/msg/composite_data.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class VnSensorMsgs : public rclcpp::Node
{
public:
  VnSensorMsgs()
      : Node("vn_sensor_msgs")
  {
    // Publishers
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);

    // Subscribers
    auto sub_vn_cb = std::bind(&VnSensorMsgs::sub_vn, this, std::placeholders::_1);
    sub_vn_ = this->create_subscription<vectornav_msgs::msg::CompositeData>("vectornav/composite_data", 10, sub_vn_cb);
  }

private:
  void sub_vn(const vectornav_msgs::msg::CompositeData::SharedPtr msg_vn) const
  {
    RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_vn->header.frame_id.c_str());

    sensor_msgs::msg::Imu msg_imu;
    msg_imu.header = msg_vn->header;
    pub_imu_->publish(msg_imu);
  }
  //
  // Member Variables
  //

  /// Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CompositeData>::SharedPtr sub_vn_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VnSensorMsgs>());
  rclcpp::shutdown();
  return 0;
}
