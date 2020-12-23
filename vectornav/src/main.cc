#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vectornav_msgs/msg/composite_data.hpp"

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

using namespace std::chrono_literals;


class Vectornav : public rclcpp::Node
{
public:
  Vectornav()
      : Node("vectornav")
  {
    //
    // Parameters
    //
    // TODO[Dereck] Add constraints to parameters

    // Device Port
    auto port = declare_parameter<std::string>("port", "/dev/vn100"); /// TODO[DERECK] /dev/ttyUSB0

    // Baud Rate
    // 5.2.6
    // 9600, 19200 38400 57600 115200
    // 128000 230400 460800 921600
    auto baud = declare_parameter<int>("baud", 115200);
    auto reconnect_ms = std::chrono::milliseconds(declare_parameter<int>("reconnect_ms", 500));

    // Async Output Type
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
    declare_parameter<int>("syncOutPolarity", vn::protocol::uart::SyncOutPolarity::SYNCOUTPOLARITY_NEGATIVE);
    declare_parameter<int>("syncOutSkipFactor", 0);
    declare_parameter<int>("syncOutPulseWidth_ns", 100000000);

    // Communication Protocol Control
    // 5.2.10
    declare_parameter<int>("serialCount", vn::protocol::uart::CountMode::COUNTMODE_NONE);
    declare_parameter<int>("serialStatus", vn::protocol::uart::StatusMode::STATUSMODE_OFF);
    declare_parameter<int>("spiCount", vn::protocol::uart::CountMode::COUNTMODE_NONE);
    declare_parameter<int>("spiStatus", vn::protocol::uart::StatusMode::STATUSMODE_OFF);
    declare_parameter<int>("serialChecksum", vn::protocol::uart::ChecksumMode::CHECKSUMMODE_CHECKSUM);
    declare_parameter<int>("spiChecksum", vn::protocol::uart::ChecksumMode::CHECKSUMMODE_OFF);
    declare_parameter<int>("errorMode", vn::protocol::uart::ErrorMode::ERRORMODE_SEND);

    // Binary Output Register 1
    // 5.2.11
    declare_parameter<int>("BO1.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_BOTH);
    declare_parameter<int>("BO1.rateDivisor", 100);
    declare_parameter<int>("BO1.commonField", 0x7FFF);
    declare_parameter<int>("BO1.timeField", vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);
    declare_parameter<int>("BO1.imuField", vn::protocol::uart::ImuGroup::IMUGROUP_NONE);
    declare_parameter<int>("BO1.gpsField", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
    declare_parameter<int>("BO1.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
    declare_parameter<int>("BO1.insField", vn::protocol::uart::InsGroup::INSGROUP_NONE);
    declare_parameter<int>("BO1.gps2Field", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // Binary Output Register 2
    // 5.2.12
    declare_parameter<int>("BO2.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_NONE);
    declare_parameter<int>("BO2.rateDivisor", 0);
    declare_parameter<int>("BO2.commonField", vn::protocol::uart::CommonGroup::COMMONGROUP_NONE);
    declare_parameter<int>("BO2.timeField", vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);
    declare_parameter<int>("BO2.imuField", vn::protocol::uart::ImuGroup::IMUGROUP_NONE);
    declare_parameter<int>("BO2.gpsField", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
    declare_parameter<int>("BO2.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
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
    declare_parameter<int>("BO3.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
    declare_parameter<int>("BO3.insField", vn::protocol::uart::InsGroup::INSGROUP_NONE);
    declare_parameter<int>("BO3.gps2Field", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // Composite Data Publisher
    pub_ = this->create_publisher<vectornav_msgs::msg::CompositeData>("vectornav/composite_data", 10);

    // Connect to the sensor
    connect(port, baud);

    // Monitor Connection
    if (reconnect_ms > 0ms)
    {
      reconnect_timer_ = create_wall_timer(reconnect_ms, std::bind(&Vectornav::reconnect_timer, this));
    }
  }

private:
  /**
   * Periodically check for connection drops and try to reconnect
   * 
   * Monitor rate is configured via the 'reconnect_ms' parameter, Set to zero to disable.
   */
  void reconnect_timer()
  {
    // Check if the sensor is connected
    if (vs_.verifySensorConnectivity())
    {
      return;
    }

    // Try to reconnect
    try
    {
      std::string port = get_parameter("port").as_string();
      int baud = get_parameter("baud").as_int();
      if(!connect(port, baud))
      {
        vs_.disconnect();
      }
    }
    catch (...)
    {
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
  bool connect(const std::string port, const int baud)
  {
    // Default response was too low and retransmit time was too long by default.
    vs_.setResponseTimeoutMs(1000); // ms
    vs_.setRetransmitDelayMs(50);   // ms

    // Check if the requested baud rate is supported
    auto baudrates = vs_.supportedBaudrates();
    if (baud > 0 && std::find(baudrates.begin(), baudrates.end(), baud) == baudrates.end())
    {
      RCLCPP_FATAL(get_logger(), "Baudrate Not Supported: %d", baud);
      return false;
    }

    // Try to connect with the requested baud rate but retry all
    // supported rates on failure
    baudrates.insert(baudrates.begin(), baud);
    for (auto b : baudrates)
    {
      try
      {
        vs_.connect(port, b);

        if (vs_.verifySensorConnectivity())
        {
          break;
        }
      }
      catch (...)
      {
        // Don't care...
      }
    }

    // Restore Factory Settings for consistency
    // TODO[Dereck] Move factoryReset to Service Call?
    // vs_.restoreFactorySettings();

    // Configure the sensor to the requested baudrate
    if (baud > 0 && baud != vs_.baudrate())
    {
      vs_.changeBaudRate(baud);
    }

    // Verify connection one more time
    if (!vs_.verifySensorConnectivity())
    {
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

    // TODO[Dereck] Move writeUserTag to Service Call?
    // 5.2.1
    // vs_.writeUserTag("");

    // Async Output Type
    // 5.2.7
    auto AsyncDataOutputType = (vn::protocol::uart::AsciiAsync)get_parameter("AsyncDataOutputType").as_int();
    vs_.writeAsyncDataOutputType(AsyncDataOutputType);

    // Async output Frequency (Hz)
    // 5.2.8
    int AsyncDataOutputFreq = get_parameter("AsyncDataOutputFrequency").as_int();
    vs_.writeAsyncDataOutputFrequency(AsyncDataOutputFreq);

    // Sync control 
    // 5.2.9
    vn::sensors::SynchronizationControlRegister configSync;
    configSync.syncInMode = (vn::protocol::uart::SyncInMode)get_parameter("syncInMode").as_int(); 
    configSync.syncInEdge = (vn::protocol::uart::SyncInEdge)get_parameter("syncInEdge").as_int();; 
    configSync.syncInSkipFactor = get_parameter("syncInSkipFactor").as_int();;
    configSync.syncOutMode = (vn::protocol::uart::SyncOutMode)get_parameter("syncOutMode").as_int();; 
    configSync.syncOutPolarity = (vn::protocol::uart::SyncOutPolarity)get_parameter("syncOutPolarity").as_int();; 
    configSync.syncOutSkipFactor = get_parameter("syncOutSkipFactor").as_int();; 
    configSync.syncOutPulseWidth = get_parameter("syncOutPulseWidth_ns").as_int();; 
    vs_.writeSynchronizationControl(configSync);

    // Communication Protocol Control
    // 5.2.10
    vn::sensors::CommunicationProtocolControlRegister configComm;
    configComm.serialCount = (vn::protocol::uart::CountMode)get_parameter("serialCount").as_int(); 
    configComm.serialStatus = (vn::protocol::uart::StatusMode)get_parameter("serialStatus").as_int(); 
    configComm.spiCount = (vn::protocol::uart::CountMode)get_parameter("spiCount").as_int(); 
    configComm.spiStatus = (vn::protocol::uart::StatusMode)get_parameter("spiStatus").as_int(); 
    configComm.serialChecksum = (vn::protocol::uart::ChecksumMode)get_parameter("serialChecksum").as_int(); 
    configComm.spiChecksum = (vn::protocol::uart::ChecksumMode)get_parameter("spiChecksum").as_int(); 
    configComm.errorMode = (vn::protocol::uart::ErrorMode)get_parameter("errorMode").as_int(); 
    vs_.writeCommunicationProtocolControl(configComm);

    // Binary Output Register 1
    // 5.2.11
    vn::sensors::BinaryOutputRegister configBO1;
    configBO1.asyncMode = (vn::protocol::uart::AsyncMode)get_parameter("BO1.asyncMode").as_int(); 
    configBO1.rateDivisor = get_parameter("BO1.rateDivisor").as_int(); 
    configBO1.commonField = (vn::protocol::uart::CommonGroup)get_parameter("BO1.commonField").as_int(); 
    configBO1.timeField = (vn::protocol::uart::TimeGroup)get_parameter("BO1.timeField").as_int(); 
    configBO1.imuField = (vn::protocol::uart::ImuGroup)get_parameter("BO1.imuField").as_int(); 
    configBO1.gpsField = (vn::protocol::uart::GpsGroup)get_parameter("BO1.gpsField").as_int(); 
    configBO1.attitudeField = (vn::protocol::uart::AttitudeGroup)get_parameter("BO1.attitudeField").as_int(); 
    configBO1.insField = (vn::protocol::uart::InsGroup)get_parameter("BO1.insField").as_int(); 
    configBO1.gps2Field = (vn::protocol::uart::GpsGroup)get_parameter("BO1.gps2Field").as_int(); 
    vs_.writeBinaryOutput1(configBO1);

    // Binary Output Register 2
    // 5.2.12
    vn::sensors::BinaryOutputRegister configBO2;
    configBO2.asyncMode = (vn::protocol::uart::AsyncMode)get_parameter("BO2.asyncMode").as_int(); 
    configBO2.rateDivisor = get_parameter("BO2.rateDivisor").as_int(); 
    configBO2.commonField = (vn::protocol::uart::CommonGroup)get_parameter("BO2.commonField").as_int(); 
    configBO2.timeField = (vn::protocol::uart::TimeGroup)get_parameter("BO2.timeField").as_int(); 
    configBO2.imuField = (vn::protocol::uart::ImuGroup)get_parameter("BO2.imuField").as_int(); 
    configBO2.gpsField = (vn::protocol::uart::GpsGroup)get_parameter("BO2.gpsField").as_int(); 
    configBO2.attitudeField = (vn::protocol::uart::AttitudeGroup)get_parameter("BO2.attitudeField").as_int(); 
    configBO2.insField = (vn::protocol::uart::InsGroup)get_parameter("BO2.insField").as_int(); 
    configBO2.gps2Field = (vn::protocol::uart::GpsGroup)get_parameter("BO2.gps2Field").as_int(); 
    vs_.writeBinaryOutput2(configBO2);

    // Binary Output Register 3
    // 5.2.13
    vn::sensors::BinaryOutputRegister configBO3;
    configBO3.asyncMode = (vn::protocol::uart::AsyncMode)get_parameter("BO3.asyncMode").as_int(); 
    configBO3.rateDivisor = get_parameter("BO3.rateDivisor").as_int(); 
    configBO3.commonField = (vn::protocol::uart::CommonGroup)get_parameter("BO3.commonField").as_int(); 
    configBO3.timeField = (vn::protocol::uart::TimeGroup)get_parameter("BO3.timeField").as_int(); 
    configBO3.imuField = (vn::protocol::uart::ImuGroup)get_parameter("BO3.imuField").as_int(); 
    configBO3.gpsField = (vn::protocol::uart::GpsGroup)get_parameter("BO3.gpsField").as_int(); 
    configBO3.attitudeField = (vn::protocol::uart::AttitudeGroup)get_parameter("BO3.attitudeField").as_int(); 
    configBO3.insField = (vn::protocol::uart::InsGroup)get_parameter("BO3.insField").as_int(); 
    configBO3.gps2Field = (vn::protocol::uart::GpsGroup)get_parameter("BO3.gps2Field").as_int(); 
    vs_.writeBinaryOutput3(configBO3);

    // Register Binary Data Callback
    vs_.registerAsyncPacketReceivedHandler(this, Vectornav::AsyncPacketReceivedHandler);

    // Connection Successful
    return true;
  }

  static void ErrorPacketReceivedHandler(void* nodeptr, vn::protocol::uart::Packet& errorPacket, size_t packetStartRunningIndex)
  {
    // Get handle to the vectornav class
    auto node = reinterpret_cast<Vectornav*>(nodeptr);

    auto err = errorPacket.parseError();

    RCLCPP_ERROR(node->get_logger(), "SensorError: %d", (int)err);
    // TODO[Dereck] Display error text
  }

  static void AsyncPacketReceivedHandler(void* nodeptr, vn::protocol::uart::Packet& asyncPacket, size_t packetStartRunningIndex)
  {
    // Get handle to the vectornav class
    auto node = reinterpret_cast<Vectornav*>(nodeptr);

    // Verify that this packet is a binary output message 
    if (asyncPacket.type() != vn::protocol::uart::Packet::TYPE_BINARY)
    {
      return;
    }

    // Groups
    auto msg = vectornav_msgs::msg::CompositeData();
    msg.groups = asyncPacket.groups();

    // Fields
    // Need to count the number of 1's in the group bitfield
    uint groupcount = countSetBits(msg.groups);

    // Copy each group field identifier to the msg
    for(int i = 0; i < groupcount; ++i)
    {
      msg.group_fields.push_back(asyncPacket.groupField(i));
    }

    // Parse data into CompositeData container
    vn::sensors::CompositeData cd = cd.parse(asyncPacket);
    parseCommonGroup(cd, msg);

    node->pub_->publish(msg);
  }

  /** Copy Common Group fields in binary packet to a CompositeData message
   *
   * \param asyncPacket Async Binary Packet
   * \param msg Vectornav CompositeData ROS Message
   */
  static void parseCommonGroup(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    if(compositeData.hasTimeStartup())
    {
      msg.common_timestartup = compositeData.timeStartup();
    }

    if(compositeData.hasTimeGps())
    {
      msg.common_timegps = compositeData.timeGps();
    }

    if(compositeData.hasTimeSyncIn())
    {
      msg.common_timesyncin = compositeData.timeSyncIn();
    }

    if(compositeData.hasYawPitchRoll())
    {
      msg.common_yawpitchroll = toMsg(compositeData.yawPitchRoll());
    }

    if(compositeData.hasQuaternion())
    {
      msg.common_quaternion = toMsg(compositeData.quaternion());
    }

    if(compositeData.hasAngularRate())
    {
      msg.common_angularrate = toMsg(compositeData.angularRate());
    }

    if(compositeData.hasPositionEstimatedLla())
    {
      msg.common_position = toMsg(compositeData.positionEstimatedLla());
    }

    if(compositeData.hasVelocityEstimatedNed())
    {
      msg.common_velocity = toMsg(compositeData.velocityEstimatedNed());
    }

    if(compositeData.hasAcceleration())
    {
      msg.common_accel = toMsg(compositeData.acceleration());
    }

    if(compositeData.hasAccelerationUncompensated())
    {
      msg.common_imu_accel = toMsg(compositeData.accelerationUncompensated());
    }

    if(compositeData.hasAngularRateUncompensated())
    {
      msg.common_imu_rate = toMsg(compositeData.angularRateUncompensated());
    }

    if(compositeData.hasMagnetic())
    {
      msg.common_magpres_mag = toMsg(compositeData.magnetic());
    }

    if(compositeData.hasTemperature())
    {
      msg.common_magpres_temp = compositeData.temperature();
    }

    if(compositeData.hasPressure())
    {
      msg.common_magpres_pres = compositeData.pressure();
    }

    if(compositeData.hasDeltaTime())
    {
      msg.common_deltatheta_dtime = compositeData.deltaTime();
    }

    if(compositeData.hasDeltaTheta())
    {
      msg.common_deltatheta_dtheta = toMsg(compositeData.deltaTheta());
    }

    if(compositeData.hasDeltaVelocity())
    {
      msg.common_deltatheta_dvel = toMsg(compositeData.deltaVelocity());
    }

    if(compositeData.hasInsStatus())
    {
      msg.common_insstatus = toMsg(compositeData.insStatus());
    }

    if(compositeData.hasSyncInCnt())
    {
      msg.common_syncincnt = compositeData.syncInCnt();
    }

    if(compositeData.hasTimeGpsPps())
    {
      msg.common_timegpspps = compositeData.timeGpsPps();
    }
  }

  /** Copy Time Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  static void parseTimeGroup(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    if(compositeData.hasTimeStartup())
    {
      msg.common_timegpspps = compositeData.timeStartup();
    }

    if(compositeData.hasTimeGps())
    {
      msg.time_timegps = compositeData.timeGps();
    }

    if(compositeData.hasGpsTow())
    {
      msg.time_gpstow = compositeData.gpsTow();
    }

    if(compositeData.hasWeek())
    {
      msg.time_gpsweek = compositeData.week();
    }

    if(compositeData.hasTimeSyncIn())
    {
      msg.time_timesyncin = compositeData.timeSyncIn();
    }

    if(compositeData.hasTimeGpsPps())
    {
      msg.time_timegpspps = compositeData.timeGpsPps();
    }

    if(compositeData.hasTimeUtc())
    {
      msg.time_timeutc = toMsg(compositeData.timeUtc());
    }

    if(compositeData.hasSyncInCnt())
    {
      msg.time_syncincnt = compositeData.syncInCnt();
    }

    if(compositeData.hasSyncOutCnt())
    {
      msg.time_syncoutcnt = compositeData.syncOutCnt();
    }

    if(compositeData.hasTimeStatus())
    {
      msg.time_timestatus = toMsg(compositeData.hasTimeStatus());
    }
  }

  /** Copy IMU Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  static void parseIMUGroup(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    // IMU Status is a reserved field, skip it.

    if(compositeData.hasMagneticUncompensated())
    {
      msg.imu_uncompmag = toMsg(compositeData.magneticUncompensated());
    }

    if(compositeData.hasAccelerationUncompensated())
    {
      msg.imu_uncompaccel = toMsg(compositeData.accelerationUncompensated());
    }

    if(compositeData.hasAngularRateUncompensated())
    {
      msg.imu_uncompgyro = toMsg(compositeData.angularRateUncompensated());
    }

    if(compositeData.hasTemperature())
    {
      msg.imu_temp = compositeData.temperature();
    }

    if(compositeData.hasPressure())
    {
      msg.imu_pres = compositeData.pressure();
    }

    if(compositeData.hasDeltaTime())
    {
      msg.imu_deltatheta_time = compositeData.deltaTime();
    }

    if(compositeData.hasDeltaTheta())
    {
      msg.imu_deltatheta_dtheta = toMsg(compositeData.deltaTheta());
    }

    if(compositeData.hasDeltaVelocity())
    {
      msg.imu_deltavel = toMsg(compositeData.deltaVelocity());
    }

    if(compositeData.hasMagnetic())
    {
      msg.imu_mag = toMsg(compositeData.magnetic());
    }

    if(compositeData.hasAcceleration())
    {
      msg.imu_accel = toMsg(compositeData.acceleration());
    }

    if(compositeData.hasAngularRate())
    {
      msg.imu_angularrate = toMsg(compositeData.angularRate());
    }

    if(compositeData.hasSensSat())
    {
      msg.imu_sensat = compositeData.sensSat();
    }
  }

  /** Copy GPS Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  static void parseGPSGroup(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    if(compositeData.hasTimeUtc())
    {
      msg.gps_utc = toMsg(compositeData.timeUtc());
    }

    if(compositeData.hasGpsTow())
    {
      msg.gps_tow = compositeData.gpsTow();
    }

    // TODO[Dereck] VNCXX is missing the read function for this field
    // if(compositeData.hasGpsWeek())
    // {
    //   msg.gps_week = compositeData.gpsWeek();
    // }

    if(compositeData.hasNumSats())
    {
      msg.gps_numsats = compositeData.numSats();
    }

    if(compositeData.hasFix())
    {
      msg.gps_fix = compositeData.fix();
    }

    if(compositeData.hasPositionGpsLla())
    {
      msg.gps_poslla = toMsg(compositeData.positionGpsLla());
    }

    if(compositeData.hasPositionGpsEcef())
    {
      msg.gps_posecef = toMsg(compositeData.positionGpsEcef());
    }

    if(compositeData.hasVelocityGpsNed())
    {
      msg.gps_velned = toMsg(compositeData.velocityGpsNed());
    }

    if(compositeData.hasVelocityGpsEcef())
    {
      msg.gps_velecef = toMsg(compositeData.velocityGpsEcef());
    }

    if(compositeData.hasPositionUncertaintyGpsNed())
    {
      msg.gps_posu = toMsg(compositeData.positionUncertaintyGpsNed());
    }

    if(compositeData.hasVelocityUncertaintyGps())
    {
      msg.gps_velu = compositeData.velocityUncertaintyGps();
    }

    if(compositeData.hasTimeUncertainty())
    {
      msg.gps_timeu = compositeData.timeUncertainty();
    }

    if(compositeData.hasTimeInfo())
    {
      msg.gps_timeinfo_status = compositeData.timeInfo().timeStatus;
      msg.gps_timeinfo_leapseconds = compositeData.timeInfo().timeStatus;
    }

    if(compositeData.hasDop())
    {
      msg.gps_dop = toMsg(compositeData.dop());
    }
  }

  /** Copy Attitude Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  static void parseAttitudeGroup(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    if(compositeData.hasVpeStatus())
    {
      msg.attitude_vpestatus = toMsg(compositeData.vpeStatus());
    }

    if(compositeData.hasYawPitchRoll())
    {
      msg.attitude_yawpitchroll = toMsg(compositeData.yawPitchRoll());
    }

    if(compositeData.hasQuaternion())
    {
      msg.attitude_quaternion = toMsg(compositeData.quaternion());
    }

    if(compositeData.hasDirectionCosineMatrix())
    {
      msg.attitude_dcm = toMsg(compositeData.directionCosineMatrix());
    }

    if(compositeData.hasMagneticNed())
    {
      msg.attitude_magned = toMsg(compositeData.magneticNed());
    }

    if(compositeData.hasAccelerationNed())
    {
      msg.attitude_accelned = toMsg(compositeData.accelerationNed());
    }

    if(compositeData.hasAccelerationLinearBody())
    {
      msg.attitude_linearaccelbody = toMsg(compositeData.accelerationLinearBody());
    }

    if(compositeData.hasAccelerationLinearNed())
    {
      msg.attitude_linearaccelned = toMsg(compositeData.accelerationLinearNed());
    }

    if(compositeData.hasAttitudeUncertainty())
    {
      msg.attitude_ypru = toMsg(compositeData.attitudeUncertainty());
    }
  }

  /** Copy INS Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   */
  static void parseInsGroup(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    if(compositeData.hasVpeStatus())
    {
      msg.ins_insstatus = toMsg(compositeData.insStatus());
    }
    
    if(compositeData.hasPositionEstimatedLla())
    {
      msg.ins_poslla = toMsg(compositeData.positionEstimatedLla());
    }
    
    if(compositeData.hasPositionEstimatedEcef())
    {
      msg.ins_posecef = toMsg(compositeData.positionEstimatedEcef());
    }
    
    if(compositeData.hasVelocityEstimatedBody())
    {
      msg.ins_velbody = toMsg(compositeData.velocityEstimatedBody());
    }
    
    if(compositeData.hasVelocityEstimatedNed())
    {
      msg.ins_velned = toMsg(compositeData.velocityEstimatedNed());
    }
    
    if(compositeData.hasVelocityEstimatedEcef())
    {
      msg.ins_velecef = toMsg(compositeData.velocityEstimatedEcef());
    }
    
    if(compositeData.hasMagneticEcef())
    {
      msg.ins_magecef = toMsg(compositeData.magneticEcef());
    }
    
    if(compositeData.hasMagneticEcef())
    {
      msg.ins_accelecef = toMsg(compositeData.magneticEcef());
    }
    
    if(compositeData.hasAccelerationEcef())
    {
      msg.ins_linearaccelecef = toMsg(compositeData.accelerationEcef());
    }
    
    if(compositeData.hasAccelerationLinearEcef())
    {
      msg.ins_linearaccelecef = toMsg(compositeData.accelerationLinearEcef());
    }
    
    if(compositeData.hasPositionUncertaintyEstimated())
    {
      msg.ins_posu = compositeData.positionUncertaintyEstimated();
    }
    
    if(compositeData.hasVelocityUncertaintyEstimated())
    {
      msg.ins_velu = compositeData.velocityUncertaintyEstimated();
    }
  }

  /** Copy GPS2 Group fields in binary packet to a CompositeData message
   *
   * \param compositeData Async Binary Packet CompositeData
   * \param msg Vectornav CompositeData ROS Message
   * 
   * TODO[Dereck] VNCXX is missing some read functions
   */
  static void parseGPS2Group(vn::sensors::CompositeData& compositeData, vectornav_msgs::msg::CompositeData& msg)
  {
    // if(compositeData.hasTimeUtc2())
    // {
    //   msg.gps2_utc = toMsg(compositeData.timeUtc2());
    // }

    if(compositeData.hasGps2Tow())
    {
      msg.gps2_tow = compositeData.gps2Tow();
    }

    // if(compositeData.hasGps2Week())
    // {
    //   msg.gps2_week = compositeData.gps2Week();
    // }

    // if(compositeData.hasNumSats2())
    // {
    //   msg.gps2_numsats = compositeData.numSats2();
    // }

    if(compositeData.hasFix2())
    {
      msg.gps2_fix = compositeData.fix2();
    }

    if(compositeData.hasPositionGps2Lla())
    {
      msg.gps2_poslla = toMsg(compositeData.positionGps2Lla());
    }

    if(compositeData.hasPositionGps2Ecef())
    {
      msg.gps2_posecef = toMsg(compositeData.positionGps2Ecef());
    }

    if(compositeData.hasVelocityGps2Ned())
    {
      msg.gps2_velned = toMsg(compositeData.velocityGps2Ned());
    }

    if(compositeData.hasVelocityGps2Ecef())
    {
      msg.gps2_velecef = toMsg(compositeData.velocityGps2Ecef());
    }

    if(compositeData.hasPositionUncertaintyGps2Ned())
    {
      msg.gps2_posu = toMsg(compositeData.positionUncertaintyGps2Ned());
    }

    if(compositeData.hasVelocityUncertaintyGps2())
    {
      msg.gps2_velu = compositeData.velocityUncertaintyGps2();
    }

    // if(compositeData.hasTimeUncertainty2())
    // {
    //   msg.gps2_timeu = compositeData.timeUncertainty2();
    // }

    // if(compositeData.hasTimeInfo2())
    // {
    //   msg.gps2_timeinfo_status = compositeData.timeInfo2().timeStatus;
    //   msg.gps2_timeinfo_leapseconds = compositeData.timeInfo2().timeStatus;
    // }

    // if(compositeData.hasDop2())
    // {
    //   msg.gps2_dop = toMsg(compositeData.dop2());
    // }
  }

  // 
  // Helper Functions
  // 

  /// Convert from vn::math::vec3f to geometry_msgs::msgs::Vector3
  static inline
  geometry_msgs::msg::Vector3 toMsg(const vn::math::vec3f& rhs)
  {
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  /// Convert from vn::math::vec4f to geometry_msgs::msgs::Quaternion
  static inline
  geometry_msgs::msg::Quaternion toMsg(const vn::math::vec4f& rhs)
  {
    geometry_msgs::msg::Quaternion lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    lhs.w = rhs[3];
    return lhs;
  }

  /// Convert from vn::math::vec3d to geometry_msgs::msgs::Point
  static inline
  geometry_msgs::msg::Point toMsg(const vn::math::vec3d& rhs)
  {
    geometry_msgs::msg::Point lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  /// Convert from vn::protocol::uart::TimeUTC to vectornav_msgs::msg::TimeUTC
  static inline
  vectornav_msgs::msg::TimeUTC toMsg(const vn::protocol::uart::TimeUtc& rhs)
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
  static inline
  vectornav_msgs::msg::DOP toMsg(const vn::protocol::uart::GnssDop& rhs)
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
  static inline
  vectornav_msgs::msg::VpeStatus toMsg(const vn::protocol::uart::VpeStatus& rhs)
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
  static inline
  std::array<float, 9> toMsg(const vn::math::mat3f& rhs)
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
  static inline
  vectornav_msgs::msg::TimeStatus toMsg(const uint8_t rhs)
  {
    vectornav_msgs::msg::TimeStatus lhs;
    lhs.time_ok = rhs & 0x01;
    lhs.date_ok = rhs & 0x02;
    lhs.utctime_ok = rhs & 0x04;
    return lhs;
  }

  /// Convert from vn::math::mat3f to vectornav_msgs::msg::TimeStatus
  // TODO[Dereck] nvcxx uses an enum to hold a bitfeild, this is likely undefined behavior
  static inline
  vectornav_msgs::msg::InsStatus toMsg(const vn::protocol::uart::InsStatus& rhs)
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


  /// Count the number of set bits in a number
  template <typename T>
  static uint countSetBits(T n) 
  {
      T count = 0;
      while (n != 0)
      {
          n = n & (n-1);
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

  /// Composite Data Publisher
  rclcpp::Publisher<vectornav_msgs::msg::CompositeData>::SharedPtr pub_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vectornav>());
  rclcpp::shutdown();
  return 0;
}
