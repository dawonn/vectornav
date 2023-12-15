/** VectorNav ROS2 Interface
 *
 * Copyright 2021 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#ifndef VECTORNAV__VECTORNAV_HPP_
#define VECTORNAV__VECTORNAV_HPP_

#include <functional>
#include <memory>
#include <string>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <vectornav_msgs/msg/attitude_group.hpp>
#include <vectornav_msgs/msg/common_group.hpp>
#include <vectornav_msgs/msg/gps_group.hpp>
#include <vectornav_msgs/msg/imu_group.hpp>
#include <vectornav_msgs/msg/ins_group.hpp>
#include <vectornav_msgs/msg/time_group.hpp>

// VectorNav libvncxx
#include "vn/compositedata.h"
#include "vn/sensors.h"

namespace vectornav {
  class Vectornav : public rclcpp::Node
  {
  public:
    explicit Vectornav(const rclcpp::NodeOptions & options);
    ~Vectornav();

  private:
    bool optimize_serial_communication(const std::string & portName);
    void reconnect_timer();
    bool connect(const std::string port, const int baud);
    rclcpp::Time getTimeStamp(vn::sensors::CompositeData & data);
    static void ErrorPacketReceivedHandler(
      void * nodeptr, vn::protocol::uart::Packet & errorPacket, size_t packetStartRunningIndex);
    static void AsyncPacketReceivedHandler(
      void * nodeptr, vn::protocol::uart::Packet & asyncPacket, size_t packetStartRunningIndex);
    //
    // Parsing functions
    //
    static void parseCommonGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    static void parseTimeGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    static void parseImuGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    static void parseGpsGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    static void parseAttitudeGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    static void parseInsGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    static void parseGps2Group(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields);
    //
    // Helper Functions
    //
    static inline geometry_msgs::msg::Vector3 toMsg(const vn::math::vec3f & rhs);
    static inline geometry_msgs::msg::Quaternion toMsg(const vn::math::vec4f & rhs);
    static inline geometry_msgs::msg::Point toMsg(const vn::math::vec3d & rhs);
    static inline vectornav_msgs::msg::TimeUTC toMsg(const vn::protocol::uart::TimeUtc & rhs);
    static inline vectornav_msgs::msg::DOP toMsg(const vn::protocol::uart::GnssDop & rhs);
    static inline vectornav_msgs::msg::VpeStatus toMsg(const vn::protocol::uart::VpeStatus & rhs);
    static inline std::array<float, 9> toMsg(const vn::math::mat3f & rhs);
    static inline vectornav_msgs::msg::TimeStatus toMsg(const uint8_t rhs);
    static inline vectornav_msgs::msg::InsStatus toMsg(const vn::protocol::uart::InsStatus & rhs);

    /// Count the number of set bits in a number
    template <typename T>
    static uint countSetBits(T n)
      {
        T count = 0;
        while (n != 0) {
          n = n & (n - 1);
          count++;
        }
        return count;
      }

    //
    // Member Variables
    //

    /// VectorNav Sensor Handle
    vn::sensors::VnSensor vs_;

    /// Reconnection Timer
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    /// Publishers
    rclcpp::Publisher<vectornav_msgs::msg::CommonGroup>::SharedPtr pub_common_;
    rclcpp::Publisher<vectornav_msgs::msg::TimeGroup>::SharedPtr pub_time_;
    rclcpp::Publisher<vectornav_msgs::msg::ImuGroup>::SharedPtr pub_imu_;
    rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr pub_gps_;
    rclcpp::Publisher<vectornav_msgs::msg::AttitudeGroup>::SharedPtr pub_attitude_;
    rclcpp::Publisher<vectornav_msgs::msg::InsGroup>::SharedPtr pub_ins_;
    rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr pub_gps2_;

    /// ROS header time stamp adjustments
    double averageTimeDifference_{0};
    bool adjustROSTimeStamp_{false};
  };
}
#endif // VECTORNAV__VECTORNAV_HPP_
