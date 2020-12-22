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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

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
    declare_parameter<int>("AsyncDataOutputFrequency", 50);

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
    declare_parameter<int>("spiChecksum", vn::protocol::uart::ChecksumMode::CHECKSUMMODE_CHECKSUM);
    declare_parameter<int>("errorMode", vn::protocol::uart::ErrorMode::ERRORMODE_SEND);

    // Binary Output Register 1
    // 5.2.11
    declare_parameter<int>("BO1.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_NONE);
    declare_parameter<int>("BO1.rateDivisor", 0);
    declare_parameter<int>("BO1.commonField", vn::protocol::uart::CommonGroup::COMMONGROUP_NONE);
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

    // TODO[Dereck] Move to Service Call?
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
    // TODO[Dereck] Log error messages from sensor

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
    vs_.writeBinaryOutput1(configBO2);

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
    vs_.writeBinaryOutput1(configBO3);

    // Register Binary Data Callback
    vs_.registerAsyncPacketReceivedHandler(this, Vectornav::AsyncPacketReceivedHandler);
    
    // Connection Successful
    return true;
  }

static void AsyncPacketReceivedHandler(void* node, vn::protocol::uart::Packet& asyncPacket, size_t packetStartRunningIndex)
{
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
  //countSetBits(msg.groups);

  // p->publish(msg);
}

private:

  /// Count the number of set bits in an int
  static void countSetBits(uint32_t n) 
  {
      uint32_t count = 0;
      while (n != 0)
      {
          n = n & (n-1);
          count++;
      }
  }

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
