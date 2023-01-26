/** VectorNav ROS2 Interface
 *
 * Copyright 2021 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// VectorNav libvncxx
#include "vn/util.h"
#include "vectornav.hpp"

using namespace std::chrono_literals;

// Assure that the serial port is set to async low latency in order to reduce delays and package pilup.
// These changes will stay effective until the device is unplugged

namespace vectornav {
Vectornav::Vectornav(const rclcpp::NodeOptions &options) : Node("vectornav", options)
  {
    //
    // Parameters
    //
    // TODO(Dereck): Add constraints to parameters

    // Device Port
    auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");

    // Baud Rate
    // 5.2.6
    // 9600, 19200 38400 57600 115200
    // 128000 230400 460800 921600
    auto baud = declare_parameter<int>("baud", 115200);
    auto reconnect_ms = std::chrono::milliseconds(declare_parameter<int>("reconnect_ms", 500));

    // Flag to adjust ROS time stamps
    adjustROSTimeStamp_ = declare_parameter<bool>("adjust_ros_timestamp", false);

    // Async Output Type (ASCII)
    // 5.2.7
    declare_parameter<int>("AsyncDataOutputType", vn::protocol::uart::AsciiAsync::VNOFF);

    // Async output Frequency (Hz)
    // 5.2.8
    // {1 2 4 5 10 20 25 40 50 100 200}
    declare_parameter<int>("AsyncDataOutputFrequency", 20);

    // Sync control
    // 5.2.9

    declare_parameter<int>("syncInMode", vn::protocol::uart::SyncInMode::SYNCINMODE_COUNT);
    declare_parameter<int>("syncInEdge", vn::protocol::uart::SyncInEdge::SYNCINEDGE_RISING);
    declare_parameter<int>("syncInSkipFactor", 0);
    declare_parameter<int>("syncOutMode", vn::protocol::uart::SyncOutMode::SYNCOUTMODE_NONE);
    declare_parameter<int>(
      "syncOutPolarity", vn::protocol::uart::SyncOutPolarity::SYNCOUTPOLARITY_NEGATIVE);
    declare_parameter<int>("syncOutSkipFactor", 0);
    declare_parameter<int>("syncOutPulseWidth_ns", 100000000);

    // Communication Protocol Control
    // 5.2.10
    declare_parameter<int>("serialCount", vn::protocol::uart::CountMode::COUNTMODE_NONE);
    declare_parameter<int>("serialStatus", vn::protocol::uart::StatusMode::STATUSMODE_OFF);
    declare_parameter<int>("spiCount", vn::protocol::uart::CountMode::COUNTMODE_NONE);
    declare_parameter<int>("spiStatus", vn::protocol::uart::StatusMode::STATUSMODE_OFF);
    declare_parameter<int>(
      "serialChecksum", vn::protocol::uart::ChecksumMode::CHECKSUMMODE_CHECKSUM);
    declare_parameter<int>("spiChecksum", vn::protocol::uart::ChecksumMode::CHECKSUMMODE_OFF);
    declare_parameter<int>("errorMode", vn::protocol::uart::ErrorMode::ERRORMODE_SEND);

    // Binary Output Register 1
    // 5.2.11
    declare_parameter<int>("BO1.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_BOTH);
    declare_parameter<int>("BO1.rateDivisor", 40);  // 20Hz
    declare_parameter<int>("BO1.commonField", 0x7FFF);
    declare_parameter<int>("BO1.timeField", vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);
    declare_parameter<int>("BO1.imuField", vn::protocol::uart::ImuGroup::IMUGROUP_NONE);
    declare_parameter<int>(
      "BO1.gpsField",
      vn::protocol::uart::GpsGroup::GPSGROUP_FIX | vn::protocol::uart::GpsGroup::GPSGROUP_POSU);
    declare_parameter<int>(
      "BO1.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
    declare_parameter<int>(
      "BO1.insField", vn::protocol::uart::InsGroup::INSGROUP_POSECEF |
                        vn::protocol::uart::InsGroup::INSGROUP_VELBODY);
    declare_parameter<int>("BO1.gps2Field", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // Binary Output Register 2
    // 5.2.12
    declare_parameter<int>("BO2.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_NONE);
    declare_parameter<int>("BO2.rateDivisor", 0);
    declare_parameter<int>("BO2.commonField", vn::protocol::uart::CommonGroup::COMMONGROUP_NONE);
    declare_parameter<int>("BO2.timeField", vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);
    declare_parameter<int>("BO2.imuField", vn::protocol::uart::ImuGroup::IMUGROUP_NONE);
    declare_parameter<int>("BO2.gpsField", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
    declare_parameter<int>(
      "BO2.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
    declare_parameter<int>("BO2.insField", vn::protocol::uart::InsGroup::INSGROUP_NONE);
    declare_parameter<int>("BO2.gps2Field", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // Binary Output Register 3
    // 5.2.13
    declare_parameter<int>("BO3.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_NONE);
    declare_parameter<int>("BO3.rateDivisor", 0);
    declare_parameter<int>("BO3.commonField", vn::protocol::uart::CommonGroup::COMMONGROUP_NONE);
    declare_parameter<int>("BO3.timeField", vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);
    declare_parameter<int>("BO3.imuField", vn::protocol::uart::ImuGroup::IMUGROUP_NONE);
    declare_parameter<int>("BO3.gpsField", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
    declare_parameter<int>(
      "BO3.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
    declare_parameter<int>("BO3.insField", vn::protocol::uart::InsGroup::INSGROUP_NONE);
    declare_parameter<int>("BO3.gps2Field", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    /// TODO(Dereck): Static Settings, read before write to protect flash memory?
    /// User Tag
    /// Magnetometer Compensation (7.2.1)
    /// Acceleration Compensation (7.2.2)
    /// Gyro Compensation (7.2.3)
    /// Reference Frame Rotation (7.2.4)
    /// IMU Filtering (7.2.5)
    /// Delta Theta Velocity Configuration (7.2.6)
    ///
    /// GPS Configuration (8.2.1)
    /// GPS Antenna A Offset (8.2.2)
    /// GPS Compass Baseline (8.2.3)

    // Message Header
    declare_parameter<std::string>("frame_id", "vectornav");

    // Composite Data Publisher
    pub_common_ =
      this->create_publisher<vectornav_msgs::msg::CommonGroup>("vectornav/raw/common", 10);
    pub_time_ = this->create_publisher<vectornav_msgs::msg::TimeGroup>("vectornav/raw/time", 10);
    pub_imu_ = this->create_publisher<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", 10);
    pub_gps_ = this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps", 10);
    pub_attitude_ =
      this->create_publisher<vectornav_msgs::msg::AttitudeGroup>("vectornav/raw/attitude", 10);
    pub_ins_ = this->create_publisher<vectornav_msgs::msg::InsGroup>("vectornav/raw/ins", 10);
    pub_gps2_ = this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps2", 10);

    if (!optimize_serial_communication(port)) {
      RCLCPP_WARN(get_logger(), "time of message delivery may be compromised!");
    }

    // Connect to the sensor
    connect(port, baud);

    // Monitor Connection
    if (reconnect_ms > 0ms) {
      reconnect_timer_ =
        create_wall_timer(reconnect_ms, std::bind(&Vectornav::reconnect_timer, this));
    }
  }

Vectornav::~Vectornav()
  {
    if (reconnect_timer_) {
      reconnect_timer_->cancel();
      reconnect_timer_.reset();
    }
    vs_.unregisterErrorPacketReceivedHandler();
    vs_.unregisterAsyncPacketReceivedHandler();
    vs_.disconnect();
  }
  bool Vectornav::optimize_serial_communication(const std::string & portName)
  {
#if __linux__ || __CYGWIN__
    const int portFd = open(portName.c_str(), O_RDWR | O_NOCTTY);

    if (portFd == -1) {
      RCLCPP_WARN(get_logger(), "Can't open port for optimization");
      return false;
    }

    struct serial_struct serial;
    ioctl(portFd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(portFd, TIOCSSERIAL, &serial);
    close(portFd);
    RCLCPP_INFO(get_logger(), "Set port to ASYNCY_LOW_LATENCY");
    return (true);
#elif
    RCLCPP_WARN(get_logger(), "Cannot set port to ASYNCY_LOW_LATENCY!");
    return (true);
#endif
  }

  /**
   * Periodically check for connection drops and try to reconnect
   *
   * Monitor rate is configured via the 'reconnect_ms' parameter, Set to zero to disable.
   */
  void Vectornav::reconnect_timer()
  {
    // Check if the sensor is connected
    if (vs_.verifySensorConnectivity()) {
      return;
    }

    // Try to reconnect
    try {
      std::string port = get_parameter("port").as_string();
      int baud = get_parameter("baud").as_int();
      if (!connect(port, baud)) {
        vs_.disconnect();
      }
    } catch (...) {
      // Don't care
    }
  }

  /**
   * Connect to a sensor
   *
   * \param port serial port path, eg /dev/ttyUSB0
   * \param baud baud rate to use, 0 for automatic
   *             Will automatically try all supported rates on failure and configure
   *             the device for the requested baud rate.
   * \return     true: OK, false: FAILURE
   */
  bool Vectornav::connect(const std::string port, const int baud)
  {
    // Default response was too low and retransmit time was too long by default.
    vs_.setResponseTimeoutMs(1000);  // ms
    vs_.setRetransmitDelayMs(50);    // ms

    // Check if the requested baud rate is supported
    auto baudrates = vs_.supportedBaudrates();
    if (baud > 0 && std::find(baudrates.begin(), baudrates.end(), baud) == baudrates.end()) {
      RCLCPP_FATAL(get_logger(), "Baudrate Not Supported: %d", baud);
      return false;
    }

    // Try to connect with the requested baud rate but retry all
    // supported rates on failure
    baudrates.insert(baudrates.begin(), baud);
    for (auto b : baudrates) {
      try {
        vs_.connect(port, b);

        if (vs_.verifySensorConnectivity()) {
          break;
        }
      } catch (...) {
        // Don't care...
      }
    }

    // Restore Factory Settings for consistency
    // TODO(Dereck): Move factoryReset to Service Call?
    // vs_.restoreFactorySettings();

    // Configure the sensor to the requested baudrate
    if (baud > 0 && baud != vs_.baudrate()) {
      vs_.changeBaudRate(baud);
    }

    // Verify connection one more time
    if (!vs_.verifySensorConnectivity()) {
      RCLCPP_ERROR(get_logger(), "Unable to connect via %s", port.c_str());
      return false;
    }

    // Query the sensor's model number.
    std::string mn = vs_.readModelNumber();
    std::string fv = vs_.readFirmwareVersion();
    uint32_t hv = vs_.readHardwareRevision();
    uint32_t sn = vs_.readSerialNumber();
    std::string ut = vs_.readUserTag();

    RCLCPP_INFO(get_logger(), "Connected to %s @ %d baud", port.c_str(), vs_.baudrate());
    RCLCPP_INFO(get_logger(), "Model: %s", mn.c_str());
    RCLCPP_INFO(get_logger(), "Firmware Version: %s", fv.c_str());
    RCLCPP_INFO(get_logger(), "Hardware Version : %d", hv);
    RCLCPP_INFO(get_logger(), "Serial Number : %d", sn);
    RCLCPP_INFO(get_logger(), "User Tag : \"%s\"", ut.c_str());

    //
    // Sensor Configuration
    //

    // Register Error Callback
    vs_.registerErrorPacketReceivedHandler(this, Vectornav::ErrorPacketReceivedHandler);

    // TODO(Dereck): Move writeUserTag to Service Call?
    // 5.2.1
    // vs_.writeUserTag("");

    // Async Output Type
    // 5.2.7
    auto AsyncDataOutputType =
      (vn::protocol::uart::AsciiAsync)get_parameter("AsyncDataOutputType").as_int();
    vs_.writeAsyncDataOutputType(AsyncDataOutputType);

    // Async output Frequency (Hz)
    // 5.2.8
    int AsyncDataOutputFreq = get_parameter("AsyncDataOutputFrequency").as_int();
    vs_.writeAsyncDataOutputFrequency(AsyncDataOutputFreq);

    // Sync control
    // 5.2.9
    vn::sensors::SynchronizationControlRegister configSync;
    configSync.syncInMode = (vn::protocol::uart::SyncInMode)get_parameter("syncInMode").as_int();
    configSync.syncInEdge = (vn::protocol::uart::SyncInEdge)get_parameter("syncInEdge").as_int();
    ;
    configSync.syncInSkipFactor = get_parameter("syncInSkipFactor").as_int();
    ;
    configSync.syncOutMode = (vn::protocol::uart::SyncOutMode)get_parameter("syncOutMode").as_int();
    ;
    configSync.syncOutPolarity =
      (vn::protocol::uart::SyncOutPolarity)get_parameter("syncOutPolarity").as_int();
    ;
    configSync.syncOutSkipFactor = get_parameter("syncOutSkipFactor").as_int();
    ;
    configSync.syncOutPulseWidth = get_parameter("syncOutPulseWidth_ns").as_int();
    ;
    vs_.writeSynchronizationControl(configSync);

    // Communication Protocol Control
    // 5.2.10
    vn::sensors::CommunicationProtocolControlRegister configComm;
    configComm.serialCount = (vn::protocol::uart::CountMode)get_parameter("serialCount").as_int();
    configComm.serialStatus =
      (vn::protocol::uart::StatusMode)get_parameter("serialStatus").as_int();
    configComm.spiCount = (vn::protocol::uart::CountMode)get_parameter("spiCount").as_int();
    configComm.spiStatus = (vn::protocol::uart::StatusMode)get_parameter("spiStatus").as_int();
    configComm.serialChecksum =
      (vn::protocol::uart::ChecksumMode)get_parameter("serialChecksum").as_int();
    configComm.spiChecksum =
      (vn::protocol::uart::ChecksumMode)get_parameter("spiChecksum").as_int();
    configComm.errorMode = (vn::protocol::uart::ErrorMode)get_parameter("errorMode").as_int();
    vs_.writeCommunicationProtocolControl(configComm);

    // Binary Output Register 1
    // 5.2.11
    vn::sensors::BinaryOutputRegister configBO1;
    configBO1.asyncMode = (vn::protocol::uart::AsyncMode)get_parameter("BO1.asyncMode").as_int();
    configBO1.rateDivisor = get_parameter("BO1.rateDivisor").as_int();
    configBO1.commonField =
      (vn::protocol::uart::CommonGroup)get_parameter("BO1.commonField").as_int();
    configBO1.timeField = (vn::protocol::uart::TimeGroup)get_parameter("BO1.timeField").as_int();
    configBO1.imuField = (vn::protocol::uart::ImuGroup)get_parameter("BO1.imuField").as_int();
    configBO1.gpsField = (vn::protocol::uart::GpsGroup)get_parameter("BO1.gpsField").as_int();
    configBO1.attitudeField =
      (vn::protocol::uart::AttitudeGroup)get_parameter("BO1.attitudeField").as_int();
    configBO1.insField = (vn::protocol::uart::InsGroup)get_parameter("BO1.insField").as_int();
    configBO1.gps2Field = (vn::protocol::uart::GpsGroup)get_parameter("BO1.gps2Field").as_int();
    vs_.writeBinaryOutput1(configBO1);

    // Binary Output Register 2
    // 5.2.12
    vn::sensors::BinaryOutputRegister configBO2;
    configBO2.asyncMode = (vn::protocol::uart::AsyncMode)get_parameter("BO2.asyncMode").as_int();
    configBO2.rateDivisor = get_parameter("BO2.rateDivisor").as_int();
    configBO2.commonField =
      (vn::protocol::uart::CommonGroup)get_parameter("BO2.commonField").as_int();
    configBO2.timeField = (vn::protocol::uart::TimeGroup)get_parameter("BO2.timeField").as_int();
    configBO2.imuField = (vn::protocol::uart::ImuGroup)get_parameter("BO2.imuField").as_int();
    configBO2.gpsField = (vn::protocol::uart::GpsGroup)get_parameter("BO2.gpsField").as_int();
    configBO2.attitudeField =
      (vn::protocol::uart::AttitudeGroup)get_parameter("BO2.attitudeField").as_int();
    configBO2.insField = (vn::protocol::uart::InsGroup)get_parameter("BO2.insField").as_int();
    configBO2.gps2Field = (vn::protocol::uart::GpsGroup)get_parameter("BO2.gps2Field").as_int();
    vs_.writeBinaryOutput2(configBO2);

    // Binary Output Register 3
    // 5.2.13
    vn::sensors::BinaryOutputRegister configBO3;
    configBO3.asyncMode = (vn::protocol::uart::AsyncMode)get_parameter("BO3.asyncMode").as_int();
    configBO3.rateDivisor = get_parameter("BO3.rateDivisor").as_int();
    configBO3.commonField =
      (vn::protocol::uart::CommonGroup)get_parameter("BO3.commonField").as_int();
    configBO3.timeField = (vn::protocol::uart::TimeGroup)get_parameter("BO3.timeField").as_int();
    configBO3.imuField = (vn::protocol::uart::ImuGroup)get_parameter("BO3.imuField").as_int();
    configBO3.gpsField = (vn::protocol::uart::GpsGroup)get_parameter("BO3.gpsField").as_int();
    configBO3.attitudeField =
      (vn::protocol::uart::AttitudeGroup)get_parameter("BO3.attitudeField").as_int();
    configBO3.insField = (vn::protocol::uart::InsGroup)get_parameter("BO3.insField").as_int();
    configBO3.gps2Field = (vn::protocol::uart::GpsGroup)get_parameter("BO3.gps2Field").as_int();
    vs_.writeBinaryOutput3(configBO3);

    try {
      // GPS Configuration
      // 8.2.1
      auto gps_config = vs_.readGpsConfiguration();
      RCLCPP_INFO(get_logger(), "GPS Mode       : %d", gps_config.mode);
      RCLCPP_INFO(get_logger(), "GPS PPS Source : %d", gps_config.ppsSource);
      /// TODO(Dereck): VnSensor::readGpsConfiguration() missing fields

      // GPS Offset
      // 8.2.2
      auto gps_offset = vs_.readGpsAntennaOffset();
      RCLCPP_INFO(
        get_logger(), "GPS Offset     : (%f, %f, %f)", gps_offset[0], gps_offset[1], gps_offset[2]);

      // GPS Compass Baseline
      // 8.2.3
      // According to dawonn, readGpsCompassBaseline is likely only available
      // on the VN-300
      if (vs_.determineDeviceFamily() == vn::sensors::VnSensor::VnSensor_Family_Vn300) {
        auto gps_baseline = vs_.readGpsCompassBaseline();
        RCLCPP_INFO(
          get_logger(), "GPS Baseline     : (%f, %f, %f), (%f, %f, %f)", gps_baseline.position[0],
          gps_baseline.position[1], gps_baseline.position[2], gps_baseline.uncertainty[0],
          gps_baseline.uncertainty[1], gps_baseline.uncertainty[2]);
      }
    } catch (const vn::sensors::sensor_error & e) {
      RCLCPP_WARN(get_logger(), "GPS initialization error (does your sensor have one?)");
    }
    // Register Binary Data Callback
    vs_.registerAsyncPacketReceivedHandler(this, Vectornav::AsyncPacketReceivedHandler);
    // Connection Successful
    return true;
  }

  /**
   * Adjust ROS header time stamp to match sensor time stamp. An exponential
   * moving average is kept of the difference between sensor time and system time.
   * This average can then be used to compute the ROS time from the sensor time.
   *
   * \param  sensorTime   time stamp received from the vectornav
   * \return  adjusted ROS time
   */

  rclcpp::Time Vectornav::getTimeStamp(vn::sensors::CompositeData & data)
  {
    const rclcpp::Time t = now();
    if (!data.hasTimeStartup() || !adjustROSTimeStamp_) {
      return (t);  // cannot or want not adjust time
    }
    const uint64_t sensorTime = data.timeStartup();
    const double dt = t.seconds() - sensorTime * 1e-9;
    if (averageTimeDifference_ == 0) {  // first call
      averageTimeDifference_ = dt;
    }

    // compute exponential moving average
    const double alpha = 0.001;  // average over rougly 1000 samples
    averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;

    // adjust sensor time by average difference to ROS time
    const rclcpp::Time adjustedTime = rclcpp::Time(sensorTime, RCL_SYSTEM_TIME) +
                                      rclcpp::Duration::from_seconds(averageTimeDifference_);
    return (adjustedTime);
  }

  void Vectornav::ErrorPacketReceivedHandler(
    void * nodeptr, vn::protocol::uart::Packet & errorPacket, size_t packetStartRunningIndex)
  {
    // Get handle to the vectornav class
    auto node = reinterpret_cast<Vectornav *>(nodeptr);

    auto err = errorPacket.parseError();

    RCLCPP_ERROR(node->get_logger(), "SensorError: %d", (int)err);
    // TODO(Dereck): Display error text
  }

  void Vectornav::AsyncPacketReceivedHandler(
    void * nodeptr, vn::protocol::uart::Packet & asyncPacket, size_t packetStartRunningIndex)
  {
    // Get handle to the vectornav class
    auto node = reinterpret_cast<Vectornav *>(nodeptr);

    // Verify that this packet is a binary output message
    if (asyncPacket.type() != vn::protocol::uart::Packet::TYPE_BINARY) {
      return;
    }

    // Parse data into CompositeData container
    vn::sensors::CompositeData cd = cd.parse(asyncPacket);

    // Groups
    auto i = 0;

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_COMMON)
      parseCommonGroup(node, cd, asyncPacket.groupField(i++));

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_TIME)
      parseTimeGroup(node, cd, asyncPacket.groupField(i++));

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_IMU)
      parseImuGroup(node, cd, asyncPacket.groupField(i++));

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_GPS)
      parseGpsGroup(node, cd, asyncPacket.groupField(i++));

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_ATTITUDE)
      parseAttitudeGroup(node, cd, asyncPacket.groupField(i++));

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_INS)
      parseInsGroup(node, cd, asyncPacket.groupField(i++));

    if (asyncPacket.groups() & vn::protocol::uart::BinaryGroup::BINARYGROUP_GPS2)
      parseGps2Group(node, cd, asyncPacket.groupField(i++));
  }

  /** Copy Common Group fields in binary packet to a CompositeData message
   *
   * \param asyncPacket Async Binary Packet
   * \param msg Vectornav CompositeData ROS Message
   */
  void Vectornav::parseCommonGroup(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::CommonGroup();

    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    if (compositeData.hasTimeStartup()) {
      msg.timestartup = compositeData.timeStartup();
    }

    if (compositeData.hasTimeGps()) {
      msg.timegps = compositeData.timeGps();
    }

    if (compositeData.hasTimeSyncIn()) {
      msg.timesyncin = compositeData.timeSyncIn();
    }

    if (compositeData.hasYawPitchRoll()) {
      msg.yawpitchroll = toMsg(compositeData.yawPitchRoll());
    }

    if (compositeData.hasQuaternion()) {
      msg.quaternion = toMsg(compositeData.quaternion());
    }

    if (compositeData.hasAngularRate()) {
      msg.angularrate = toMsg(compositeData.angularRate());
    }

    if (compositeData.hasPositionEstimatedLla()) {
      msg.position = toMsg(compositeData.positionEstimatedLla());
    }

    if (compositeData.hasVelocityEstimatedNed()) {
      msg.velocity = toMsg(compositeData.velocityEstimatedNed());
    }

    if (compositeData.hasAcceleration()) {
      msg.accel = toMsg(compositeData.acceleration());
    }

    if (compositeData.hasAccelerationUncompensated()) {
      msg.imu_accel = toMsg(compositeData.accelerationUncompensated());
    }

    if (compositeData.hasAngularRateUncompensated()) {
      msg.imu_rate = toMsg(compositeData.angularRateUncompensated());
    }

    if (compositeData.hasMagnetic()) {
      msg.magpres_mag = toMsg(compositeData.magnetic());
    }

    if (compositeData.hasTemperature()) {
      msg.magpres_temp = compositeData.temperature();
    }

    if (compositeData.hasPressure()) {
      msg.magpres_pres = compositeData.pressure();
    }

    if (compositeData.hasDeltaTime()) {
      msg.deltatheta_dtime = compositeData.deltaTime();
    }

    if (compositeData.hasDeltaTheta()) {
      msg.deltatheta_dtheta = toMsg(compositeData.deltaTheta());
    }

    if (compositeData.hasDeltaVelocity()) {
      msg.deltatheta_dvel = toMsg(compositeData.deltaVelocity());
    }

    if (compositeData.hasInsStatus()) {
      msg.insstatus = toMsg(compositeData.insStatus());
    }

    if (compositeData.hasSyncInCnt()) {
      msg.syncincnt = compositeData.syncInCnt();
    }

    if (compositeData.hasTimeGpsPps()) {
      msg.timegpspps = compositeData.timeGpsPps();
    }

    // Publish
    node->pub_common_->publish(msg);
  }

  /** Copy Time Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  void Vectornav::parseTimeGroup(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::TimeGroup();

    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    if (compositeData.hasTimeStartup()) {
      msg.timestartup = compositeData.timeStartup();
    }

    if (compositeData.hasTimeGps()) {
      msg.timegps = compositeData.timeGps();
    }

    if (compositeData.hasGpsTow()) {
      msg.gpstow = compositeData.gpsTow();
    }

    if (compositeData.hasWeek()) {
      msg.gpsweek = compositeData.week();
    }

    if (compositeData.hasTimeSyncIn()) {
      msg.timesyncin = compositeData.timeSyncIn();
    }

    if (compositeData.hasTimeGpsPps()) {
      msg.timegpspps = compositeData.timeGpsPps();
    }

    if (compositeData.hasTimeUtc()) {
      msg.timeutc = toMsg(compositeData.timeUtc());
    }

    if (compositeData.hasSyncInCnt()) {
      msg.syncincnt = compositeData.syncInCnt();
    }

    if (compositeData.hasSyncOutCnt()) {
      msg.syncoutcnt = compositeData.syncOutCnt();
    }

    if (compositeData.hasTimeStatus()) {
      msg.timestatus = toMsg(compositeData.hasTimeStatus());
    }

    // Publish
    node->pub_time_->publish(msg);
  }

  /** Copy IMU Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  void Vectornav::parseImuGroup(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::ImuGroup();
    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    // IMU Status is a reserved field, skip it.

    if (compositeData.hasMagneticUncompensated()) {
      msg.uncompmag = toMsg(compositeData.magneticUncompensated());
    }

    if (compositeData.hasAccelerationUncompensated()) {
      msg.uncompaccel = toMsg(compositeData.accelerationUncompensated());
    }

    if (compositeData.hasAngularRateUncompensated()) {
      msg.uncompgyro = toMsg(compositeData.angularRateUncompensated());
    }

    if (compositeData.hasTemperature()) {
      msg.temp = compositeData.temperature();
    }

    if (compositeData.hasPressure()) {
      msg.pres = compositeData.pressure();
    }

    if (compositeData.hasDeltaTime()) {
      msg.deltatheta_time = compositeData.deltaTime();
    }

    if (compositeData.hasDeltaTheta()) {
      msg.deltatheta_dtheta = toMsg(compositeData.deltaTheta());
    }

    if (compositeData.hasDeltaVelocity()) {
      msg.deltavel = toMsg(compositeData.deltaVelocity());
    }

    if (compositeData.hasMagnetic()) {
      msg.mag = toMsg(compositeData.magnetic());
    }

    if (compositeData.hasAcceleration()) {
      msg.accel = toMsg(compositeData.acceleration());
    }

    if (compositeData.hasAngularRate()) {
      msg.angularrate = toMsg(compositeData.angularRate());
    }

    if (compositeData.hasSensSat()) {
      msg.sensat = compositeData.sensSat();
    }

    // Publish
    node->pub_imu_->publish(msg);
  }

  /** Copy GPS Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  void Vectornav::parseGpsGroup(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::GpsGroup();

    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    if (compositeData.hasTimeUtc()) {
      msg.utc = toMsg(compositeData.timeUtc());
    }

    if (compositeData.hasGpsTow()) {
      msg.tow = compositeData.gpsTow();
    }

    // TODO(Dereck): VNCXX is missing the read function for this field
    // if(compositeData.hasGpsWeek())
    // {
    //   msg.week = compositeData.gpsWeek();
    // }

    if (compositeData.hasNumSats()) {
      msg.numsats = compositeData.numSats();
    }

    if (compositeData.hasFix()) {
      msg.fix = compositeData.fix();
    }

    if (compositeData.hasPositionGpsLla()) {
      msg.poslla = toMsg(compositeData.positionGpsLla());
    }

    // TODO(Dereck): VNCXX is missing the read function for this field
    // if(compositeData.hasPositionGpsEcef())
    // {
    //   msg.posecef = toMsg(compositeData.positionGpsEcef());
    // }

    if (compositeData.hasVelocityGpsNed()) {
      msg.velned = toMsg(compositeData.velocityGpsNed());
    }

    if (compositeData.hasVelocityGpsEcef()) {
      msg.velecef = toMsg(compositeData.velocityGpsEcef());
    }

    if (compositeData.hasPositionUncertaintyGpsNed()) {
      msg.posu = toMsg(compositeData.positionUncertaintyGpsNed());
    }

    if (compositeData.hasVelocityUncertaintyGps()) {
      msg.velu = compositeData.velocityUncertaintyGps();
    }

    if (compositeData.hasTimeUncertainty()) {
      msg.timeu = compositeData.timeUncertainty();
    }

    if (compositeData.hasTimeInfo()) {
      msg.timeinfo_status = compositeData.timeInfo().timeStatus;
      msg.timeinfo_leapseconds = compositeData.timeInfo().timeStatus;
    }

    if (compositeData.hasDop()) {
      msg.dop = toMsg(compositeData.dop());
    }

    // Publish
    node->pub_gps_->publish(msg);
  }

  /** Copy Attitude Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  void Vectornav::parseAttitudeGroup(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::AttitudeGroup();

    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    if (compositeData.hasVpeStatus()) {
      msg.vpestatus = toMsg(compositeData.vpeStatus());
    }

    if (compositeData.hasYawPitchRoll()) {
      msg.yawpitchroll = toMsg(compositeData.yawPitchRoll());
    }

    if (compositeData.hasQuaternion()) {
      msg.quaternion = toMsg(compositeData.quaternion());
    }

    if (compositeData.hasDirectionCosineMatrix()) {
      msg.dcm = toMsg(compositeData.directionCosineMatrix());
    }

    if (compositeData.hasMagneticNed()) {
      msg.magned = toMsg(compositeData.magneticNed());
    }

    if (compositeData.hasAccelerationNed()) {
      msg.accelned = toMsg(compositeData.accelerationNed());
    }

    if (compositeData.hasAccelerationLinearBody()) {
      msg.linearaccelbody = toMsg(compositeData.accelerationLinearBody());
    }

    if (compositeData.hasAccelerationLinearNed()) {
      msg.linearaccelned = toMsg(compositeData.accelerationLinearNed());
    }

    if (compositeData.hasAttitudeUncertainty()) {
      msg.ypru = toMsg(compositeData.attitudeUncertainty());
    }

    // Publish
    node->pub_attitude_->publish(msg);
  }

  /** Copy INS Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  void Vectornav::parseInsGroup(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::InsGroup();

    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    if (compositeData.hasInsStatus()) {
      msg.insstatus = toMsg(compositeData.insStatus());
    }

    if (compositeData.hasPositionEstimatedLla()) {
      msg.poslla = toMsg(compositeData.positionEstimatedLla());
    }

    if (compositeData.hasPositionEstimatedEcef()) {
      msg.posecef = toMsg(compositeData.positionEstimatedEcef());
    }

    if (compositeData.hasVelocityEstimatedBody()) {
      msg.velbody = toMsg(compositeData.velocityEstimatedBody());
    }

    if (compositeData.hasVelocityEstimatedNed()) {
      msg.velned = toMsg(compositeData.velocityEstimatedNed());
    }

    if (compositeData.hasVelocityEstimatedEcef()) {
      msg.velecef = toMsg(compositeData.velocityEstimatedEcef());
    }

    if (compositeData.hasMagneticEcef()) {
      msg.magecef = toMsg(compositeData.magneticEcef());
    }

    if (compositeData.hasMagneticEcef()) {
      msg.accelecef = toMsg(compositeData.magneticEcef());
    }

    if (compositeData.hasAccelerationEcef()) {
      msg.linearaccelecef = toMsg(compositeData.accelerationEcef());
    }

    if (compositeData.hasAccelerationLinearEcef()) {
      msg.linearaccelecef = toMsg(compositeData.accelerationLinearEcef());
    }

    if (compositeData.hasPositionUncertaintyEstimated()) {
      msg.posu = compositeData.positionUncertaintyEstimated();
    }

    if (compositeData.hasVelocityUncertaintyEstimated()) {
      msg.velu = compositeData.velocityUncertaintyEstimated();
    }

    // Publish
    node->pub_ins_->publish(msg);
  }

  /** Copy GPS2 Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   *
   * TODO(Dereck): VNCXX is missing some read functions
   */
  void Vectornav::parseGps2Group(
    Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields)
  {
    // Message to Send
    auto msg = vectornav_msgs::msg::GpsGroup();

    // Header
    msg.header.stamp = node->getTimeStamp(compositeData);
    msg.header.frame_id = node->get_parameter("frame_id").as_string();

    // Group Fields
    msg.group_fields = groupFields;

    // if(compositeData.hasTimeUtc2())
    // {
    //   msg.utc = toMsg(compositeData.timeUtc2());
    // }

    if (compositeData.hasGps2Tow()) {
      msg.tow = compositeData.gps2Tow();
    }

    // if(compositeData.hasGps2Week())
    // {
    //   msg.week = compositeData.gps2Week();
    // }

    // if(compositeData.hasNumSats2())
    // {
    //   msg.numsats = compositeData.numSats2();
    // }

    if (compositeData.hasFix2()) {
      msg.fix = compositeData.fix2();
    }

    if (compositeData.hasPositionGps2Lla()) {
      msg.poslla = toMsg(compositeData.positionGps2Lla());
    }

    if (compositeData.hasPositionGps2Ecef()) {
      msg.posecef = toMsg(compositeData.positionGps2Ecef());
    }

    if (compositeData.hasVelocityGps2Ned()) {
      msg.velned = toMsg(compositeData.velocityGps2Ned());
    }

    if (compositeData.hasVelocityGps2Ecef()) {
      msg.velecef = toMsg(compositeData.velocityGps2Ecef());
    }

    if (compositeData.hasPositionUncertaintyGps2Ned()) {
      msg.posu = toMsg(compositeData.positionUncertaintyGps2Ned());
    }

    if (compositeData.hasVelocityUncertaintyGps2()) {
      msg.velu = compositeData.velocityUncertaintyGps2();
    }

    // if(compositeData.hasTimeUncertainty2())
    // {
    //   msg.timeu = compositeData.timeUncertainty2();
    // }

    // if(compositeData.hasTimeInfo2())
    // {
    //   msg.timeinfo_status = compositeData.timeInfo2().timeStatus;
    //   msg.timeinfo_leapseconds = compositeData.timeInfo2().timeStatus;
    // }

    // if(compositeData.hasDop2())
    // {
    //   msg.dop = toMsg(compositeData.dop2());
    // }

    // Publish
    node->pub_gps2_->publish(msg);
  }

  //
  // Helper Functions
  //

  /// Convert from vn::math::vec3f to geometry_msgs::msgs::Vector3
  geometry_msgs::msg::Vector3 Vectornav::toMsg(const vn::math::vec3f & rhs)
  {
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  /// Convert from vn::math::vec4f to geometry_msgs::msgs::Quaternion
  geometry_msgs::msg::Quaternion Vectornav::toMsg(const vn::math::vec4f & rhs)
  {
    geometry_msgs::msg::Quaternion lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    lhs.w = rhs[3];
    return lhs;
  }

  /// Convert from vn::math::vec3d to geometry_msgs::msgs::Point
  geometry_msgs::msg::Point Vectornav::toMsg(const vn::math::vec3d & rhs)
  {
    geometry_msgs::msg::Point lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  /// Convert from vn::protocol::uart::TimeUTC to vectornav_msgs::msg::TimeUTC
  vectornav_msgs::msg::TimeUTC Vectornav::toMsg(const vn::protocol::uart::TimeUtc & rhs)
  {
    vectornav_msgs::msg::TimeUTC lhs;
    lhs.year = rhs.year;
    lhs.month = rhs.month;
    lhs.day = rhs.day;
    lhs.hour = rhs.hour;
    lhs.min = rhs.min;
    lhs.sec = rhs.sec;
    lhs.ms = rhs.ms;
    return lhs;
  }

  /// Convert from vn::protocol::uart::TimeUTC to vectornav_msgs::msg::TimeUTC
  vectornav_msgs::msg::DOP Vectornav::toMsg(const vn::protocol::uart::GnssDop & rhs)
  {
    vectornav_msgs::msg::DOP lhs;
    lhs.g = rhs.gDop;
    lhs.p = rhs.pDop;
    lhs.t = rhs.tDop;
    lhs.v = rhs.vDop;
    lhs.h = rhs.hDop;
    lhs.n = rhs.nDop;
    lhs.e = rhs.eDop;
    return lhs;
  }

  /// Convert from vn::protocol::uart::VpeStatus to vectornav_msgs::msg::VpeStatus
  vectornav_msgs::msg::VpeStatus Vectornav::toMsg(const vn::protocol::uart::VpeStatus & rhs)
  {
    vectornav_msgs::msg::VpeStatus lhs;
    lhs.attitude_quality = rhs.attitudeQuality;
    lhs.gyro_saturation = rhs.gyroSaturation;
    lhs.gyro_saturation_recovery = rhs.gyroSaturationRecovery;
    lhs.mag_disturbance = rhs.magDisturbance;
    lhs.mag_saturation = rhs.magSaturation;
    lhs.acc_disturbance = rhs.accDisturbance;
    lhs.acc_saturation = rhs.accSaturation;
    lhs.known_mag_disturbance = rhs.knownMagDisturbance;
    lhs.known_accel_disturbance = rhs.knownAccelDisturbance;
    return lhs;
  }

  /// Convert from vn::math::mat3f to std::array<float, 9>
  std::array<float, 9> Vectornav::toMsg(const vn::math::mat3f & rhs)
  {
    std::array<float, 9> lhs;
    lhs[0] = rhs(0, 0);
    lhs[1] = rhs(0, 1);
    lhs[2] = rhs(0, 2);
    lhs[3] = rhs(1, 0);
    lhs[4] = rhs(1, 1);
    lhs[5] = rhs(1, 2);
    lhs[6] = rhs(2, 0);
    lhs[7] = rhs(2, 1);
    lhs[8] = rhs(2, 2);
    return lhs;
  }

  /// Convert from vn::math::mat3f to vectornav_msgs::msg::TimeStatus
  vectornav_msgs::msg::TimeStatus Vectornav::toMsg(const uint8_t rhs)
  {
    vectornav_msgs::msg::TimeStatus lhs;
    lhs.time_ok = rhs & 0x01;
    lhs.date_ok = rhs & 0x02;
    lhs.utctime_ok = rhs & 0x04;
    return lhs;
  }

  /// Convert from vn::math::mat3f to vectornav_msgs::msg::TimeStatus
  // TODO(Dereck): vncxx uses an enum to hold a bitfeild, this is likely undefined behavior
  vectornav_msgs::msg::InsStatus Vectornav::toMsg(const vn::protocol::uart::InsStatus & rhs)
  {
    vectornav_msgs::msg::InsStatus lhs;
    lhs.mode = rhs & 0x0003;
    lhs.gps_fix = rhs & vn::protocol::uart::InsStatus::INSSTATUS_GPS_FIX;
    lhs.time_error = rhs & vn::protocol::uart::InsStatus::INSSTATUS_TIME_ERROR;
    lhs.imu_error = rhs & vn::protocol::uart::InsStatus::INSSTATUS_IMU_ERROR;
    lhs.time_error = rhs & vn::protocol::uart::InsStatus::INSSTATUS_GPS_FIX;
    lhs.mag_pres_error = rhs & vn::protocol::uart::InsStatus::INSSTATUS_MAG_PRES_ERROR;
    lhs.gps_error = rhs & vn::protocol::uart::InsStatus::INSSTATUS_GPS_ERROR;
    lhs.gps_heading_ins = rhs & 0x0080;
    lhs.gps_compass = rhs & 0x0100;
    return lhs;
  }
}
RCLCPP_COMPONENTS_REGISTER_NODE(vectornav::Vectornav)
