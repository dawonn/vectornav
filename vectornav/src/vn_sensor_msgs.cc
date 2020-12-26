#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vectornav_msgs/msg/composite_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class VnSensorMsgs : public rclcpp::Node
{
public:
  VnSensorMsgs()
      : Node("vn_sensor_msgs")
  {
    // Parameters
    declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
    declare_parameter<std::vector<double>>("angular_velocity_covariance", angular_velocity_covariance_);
    declare_parameter<std::vector<double>>("linear_acceleration_covariance", linear_acceleration_covariance_);
    declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);

    // Publishers
    //  TODO[Dereck]: Only publish data present in the composite data message
    pub_time_startup_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_startup", 10);
    pub_time_gps_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_gps", 10);
    pub_time_syncin_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_syncin", 10);
    pub_time_pps_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_pps", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
    pub_gnss_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("vectornav/gnss", 10);
    pub_imu_uncompensated_ = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu_uncompensated", 10);
    pub_magnetic_ = this->create_publisher<sensor_msgs::msg::MagneticField>("vectornav/magnetic", 10);
    pub_temperature_ = this->create_publisher<sensor_msgs::msg::Temperature>("vectornav/temperature", 10);
    pub_pressure_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("vectornav/pressure", 10);
    pub_velocity_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("vectornav/velocity_body", 10);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vectornav/pose", 10);

    // Subscribers
    auto sub_vn_cb = std::bind(&VnSensorMsgs::sub_vn, this, std::placeholders::_1);
    sub_vn_ = this->create_subscription<vectornav_msgs::msg::CompositeData>("vectornav/composite_data", 10, sub_vn_cb);
  }

private:
  /** Convert VN composite data to ROS2 standard message types
   *
   *  TODO[Dereck]: Add _ned topics as an optional feature
   */
  void sub_vn(const vectornav_msgs::msg::CompositeData::SharedPtr msg_vn) const
  {
    // RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_vn->header.frame_id.c_str());

    // Time Reference (Startup)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_vn->header;

      msg.time_ref.sec = std::floor(msg_vn->common_timestartup / 1e9);
      msg.time_ref.nanosec = msg_vn->common_timestartup - (msg.time_ref.sec * 1e9);

      msg.source = "startup";

      pub_time_startup_->publish(msg);
    }

    // Time Reference (GPS)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_vn->header;

      msg.time_ref.sec = std::floor(msg_vn->common_timegps / 1e9);
      msg.time_ref.nanosec = msg_vn->common_timegps - (msg.time_ref.sec * 1e9);

      msg.source = "gps";

      pub_time_gps_->publish(msg);
    }


    // Time Reference (SyncIn)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_vn->header;

      msg.time_ref.sec = std::floor(msg_vn->common_timesyncin / 1e9);
      msg.time_ref.nanosec = msg_vn->common_timesyncin - (msg.time_ref.sec * 1e9);

      msg.source = "syncin";

      pub_time_syncin_->publish(msg);
    }

    // Time Reference (PPS)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_vn->header;

      msg.time_ref.sec = std::floor(msg_vn->common_timegpspps / 1e9);
      msg.time_ref.nanosec = msg_vn->common_timegpspps - (msg.time_ref.sec * 1e9);

      msg.source = "pps";

      pub_time_pps_->publish(msg);
    }


    // IMU
    {
      sensor_msgs::msg::Imu msg;
      msg.header = msg_vn->header;

      // Quaternion NED -> ENU
      tf2::Quaternion q, q_ned2enu;
      fromMsg(msg_vn->common_quaternion, q);
      q_ned2enu.setRPY(M_PI,0.0,M_PI/2);
      msg.orientation = toMsg(q_ned2enu*q);

      msg.angular_velocity = msg_vn->common_angularrate;
      msg.linear_acceleration = msg_vn->common_accel;

      fill_covariance_from_param("orientation_covariance", msg.orientation_covariance);
      fill_covariance_from_param("angular_velocity_covariance", msg.angular_velocity_covariance);
      fill_covariance_from_param("linear_acceleration_covariance", msg.linear_acceleration_covariance);

      pub_imu_->publish(msg);
    }

    // GNSS
    {
      sensor_msgs::msg::NavSatFix msg;
      msg.header = msg_vn->header;

      // Status
      if(msg_vn->gps_fix == vectornav_msgs::msg::CompositeData::GPSFIX_NOFIX)
      {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }
      else
      {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      }
      
      // Position
      msg.latitude = msg_vn->common_position.x;
      msg.longitude = msg_vn->common_position.y;
      msg.altitude = msg_vn->common_position.z;

      // Covariance (Converts NED to ENU)
      // TODO[Dereck] Use DOP for better estimate?
      const std::vector<double> orientation_covariance_ = {
        msg_vn->gps_posu.y, 0.0000, 0.0000,
        0.0000, msg_vn->gps_posu.x, 0.0000,
        0.0000, 0.0000, msg_vn->gps_posu.z};

      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      pub_gnss_->publish(msg);
    }

    
    // Velocity
    {
      geometry_msgs::msg::TwistWithCovarianceStamped msg;
      msg.header = msg_vn->header;
      msg.twist.twist.linear = msg_vn->ins_velbody;
      msg.twist.twist.angular = msg_vn->common_angularrate;

      // TODO[Dereck] Velocity Covariance

      pub_velocity_->publish(msg);
    }

    
    // Pose (ECEF)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header = msg_vn->header;
      msg.header.frame_id = "earth";
      msg.pose.pose.position = msg_vn->ins_posecef;

      // Converts Quaternion in NED to ECEF
      tf2::Quaternion q, q_enu2ecef, q_ned2enu;
      q_ned2enu.setRPY(M_PI,0.0,M_PI/2);
      
      auto latitude = deg2rad(msg_vn->ins_poslla.x);
      auto longitude = deg2rad(msg_vn->ins_poslla.y);
      q_enu2ecef.setRPY(0.0, latitude, longitude);

      fromMsg(msg_vn->attitude_quaternion, q);

      msg.pose.pose.orientation = toMsg(q_ned2enu * q_enu2ecef * q);

      // TODO[Dereck] Pose Covariance

      pub_pose_->publish(msg);
    }

    // IMU (Uncompensated)
    {
      sensor_msgs::msg::Imu msg;
      msg.header = msg_vn->header;
      msg.angular_velocity = msg_vn->common_imu_rate;
      msg.linear_acceleration = msg_vn->common_imu_accel;

      fill_covariance_from_param("angular_velocity_covariance", msg.angular_velocity_covariance);
      fill_covariance_from_param("linear_acceleration_covariance", msg.linear_acceleration_covariance);

      pub_imu_uncompensated_->publish(msg);
    }

    // Magnetic Field
    {
      sensor_msgs::msg::MagneticField msg;
      msg.header = msg_vn->header;
      msg.magnetic_field = msg_vn->common_magpres_mag;

      fill_covariance_from_param("magnetic_covariance", msg.magnetic_field_covariance);

      pub_magnetic_->publish(msg);
    }

    // Temperature
    {
      sensor_msgs::msg::Temperature msg;
      msg.header = msg_vn->header;
      msg.temperature = msg_vn->common_magpres_temp;

      pub_temperature_->publish(msg);
    }

    // Pressure
    {
      sensor_msgs::msg::FluidPressure msg;
      msg.header = msg_vn->header;
      msg.fluid_pressure = msg_vn->common_magpres_pres * 1e3; // kPa to Pa

      pub_pressure_->publish(msg);
    }


  }

  /** Copy a covariance matrix array from a parameter into a msg array
   * 
   * If a single value is provided, this will set the diagonal values
   * If three values are provided, this will set the diagonal values
   * If nine values are provided, this will fill the matrix
   * 
   * \param param_name Name of the parameter to read
   * \param array Array to fill
   */
  void fill_covariance_from_param(std::string param_name, std::array<double, 9> &array) const
  {
    auto covariance = get_parameter(param_name).as_double_array();

    auto length = covariance.size();
    switch (length)
    {
    case 1:
      array[0] = covariance[0];
      array[3] = covariance[0];
      array[8] = covariance[0];
      break;

    case 3:
      array[0] = covariance[0];
      array[3] = covariance[1];
      array[8] = covariance[3];
      break;

    case 9:
      std::copy(covariance.begin(), covariance.end(), array.begin());
      break;

    default:
      RCLCPP_ERROR(get_logger(), "Parameter '%s' length is %d; expected 1, 3, or 9", param_name.c_str(), length);
    }
  }

  /// Convert from DEG to RAD
  inline static double deg2rad(double in)
  {
    return in * M_PI / 180.0;
  }

  //
  // Member Variables
  //

  /// Publishers
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_startup_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_gps_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_syncin_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_pps_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_uncompensated_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_magnetic_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_pressure_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CompositeData>::SharedPtr sub_vn_;

  /// Default orientation Covariance
  const std::vector<double> orientation_covariance_ = {
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000};

  /// Default angular_velocity Covariance
  const std::vector<double> angular_velocity_covariance_ = {
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000};

  /// Default linear_acceleration Covariance
  const std::vector<double> linear_acceleration_covariance_ = {
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000};

  /// Default magnetic field Covariance
  const std::vector<double> magnetic_field_covariance_ = {
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000,
      0.0000, 0.0000, 0.0000};

  /// TODO[Dereck] Find default covariance values
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VnSensorMsgs>());
  rclcpp::shutdown();
  return 0;
}
