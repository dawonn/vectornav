/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <cmath>
#include <iostream>

// No need to define PI twice if we already have it included...
//#define M_PI 3.14159265358979323846  /* M_PI */

// ROS Libraries
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vectornav/Ins.h>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "std_srvs/Empty.h"

ros::Publisher pubIMU, pubMag, pubGPS, pubOdom, pubTemp, pubPres, pubIns;
ros::ServiceServer resetOdomSrv;

XmlRpc::XmlRpcValue rpc_temp;

// Include this header file to get access to VectorNav sensors.
#include "vn/compositedata.h"
#include "vn/sensors.h"
#include "vn/util.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index);

// Custom user data to pass to packet callback function
struct UserData
{
  // the vectornav device identifier
  int device_family;
  // frame id used only for Odom header.frame_id
  std::string map_frame_id;
  // frame id used for header.frame_id of other messages and for Odom child_frame_id
  std::string frame_id;
  // Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
  bool tf_ned_to_enu;
  bool frame_based_enu;
  // Initial position after getting a GPS fix.
  vec3d initial_position;
  bool initial_position_set = false;

  //Unused covariances initialized to zero's
  boost::array<double, 9ul> linear_accel_covariance = {};
  boost::array<double, 9ul> angular_vel_covariance = {};
  boost::array<double, 9ul> orientation_covariance = {};

  // ROS header time stamp adjustments
  double average_time_difference{0};
  ros::Time ros_start_time;
  bool adjust_ros_timestamp{false};

  // strides
  unsigned int imu_stride;
  unsigned int output_stride;
};

// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc)
{
  // Output covariance vector
  boost::array<double, 9ul> output = {0.0};

  // Convert the RPC message to array
  ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < 9; i++) {
    ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    output[i] = (double)rpc[i];
  }
  return output;
}

// Reset initial position to current position
bool resetOdom(
  std_srvs::Empty::Request & req, std_srvs::Empty::Response & resp, UserData * user_data)
{
  ROS_INFO("Reset Odometry");
  user_data->initial_position_set = false;
  return true;
}

// Assure that the serial port is set to async low latency in order to reduce delays and package pilup.
// These changes will stay effective until the device is unplugged
#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
bool optimize_serial_communication(std::string portName)
{
  int portFd = -1;

  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    ROS_WARN("Can't open port for optimization");
    return false;
  }

  ROS_INFO("Set port to ASYNCY_LOW_LATENCY");
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}
#elif
bool optimize_serial_communication(str::string portName) { return true; }
#endif

int main(int argc, char * argv[])
{
  // keeping all information passed to callback
  UserData user_data;

  // ROS node init
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
  pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
  pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
  pubOdom = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);
  pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
  pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);
  pubIns = n.advertise<vectornav::Ins>("vectornav/INS", 1000);

  resetOdomSrv = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "reset_odom", boost::bind(&resetOdom, _1, _2, &user_data));

  // Serial Port Settings
  string SensorPort;
  int SensorBaudrate;
  int async_output_rate;
  int imu_output_rate;

  // Sensor IMURATE (800Hz by default, used to configure device)
  int SensorImuRate;

  // Load all params
  pn.param<std::string>("map_frame_id", user_data.map_frame_id, "map");
  pn.param<std::string>("frame_id", user_data.frame_id, "vectornav");
  pn.param<bool>("tf_ned_to_enu", user_data.tf_ned_to_enu, false);
  pn.param<bool>("frame_based_enu", user_data.frame_based_enu, false);
  pn.param<bool>("adjust_ros_timestamp", user_data.adjust_ros_timestamp, false);
  pn.param<int>("async_output_rate", async_output_rate, 40);
  pn.param<int>("imu_output_rate", imu_output_rate, async_output_rate);
  pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
  pn.param<int>("serial_baud", SensorBaudrate, 115200);
  pn.param<int>("fixed_imu_rate", SensorImuRate, 800);

  //Call to set covariances
  if (pn.getParam("linear_accel_covariance", rpc_temp)) {
    user_data.linear_accel_covariance = setCov(rpc_temp);
  }
  if (pn.getParam("angular_vel_covariance", rpc_temp)) {
    user_data.angular_vel_covariance = setCov(rpc_temp);
  }
  if (pn.getParam("orientation_covariance", rpc_temp)) {
    user_data.orientation_covariance = setCov(rpc_temp);
  }

  ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

  // try to optimize the serial port
  optimize_serial_communication(SensorPort);

  // Create a VnSensor object and connect to sensor
  VnSensor vs;

  // Default baudrate variable
  int defaultBaudrate;
  // Run through all of the acceptable baud rates until we are connected
  // Looping in case someone has changed the default
  bool baudSet = false;
  // Lets add the set baudrate to the top of the list, so that it will try
  // to connect with that value first (speed initialization up)
  std::vector<unsigned int> supportedBaudrates = vs.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), SensorBaudrate);
  while (!baudSet) {
    // Make this variable only accessible in the while loop
    static int i = 0;
    defaultBaudrate = supportedBaudrates[i];
    ROS_INFO("Connecting with default at %d", defaultBaudrate);
    // Default response was too low and retransmit time was too long by default.
    // They would cause errors
    vs.setResponseTimeoutMs(1000);  // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);    // Retransmit every 50 ms

    // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
    // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
    // All other values seem to work fine.
    try {
      // Connect to sensor at it's default rate
      if (defaultBaudrate != 128000 && SensorBaudrate != 128000) {
        vs.connect(SensorPort, defaultBaudrate);
        // Issues a change baudrate to the VectorNav sensor and then
        // reconnects the attached serial port at the new baudrate.
        vs.changeBaudRate(SensorBaudrate);
        // Only makes it here once we have the default correct
        ROS_INFO("Connected baud rate is %d", vs.baudrate());
        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...) {
      // Disconnect if we had the wrong default and we were connected
      vs.disconnect();
      ros::Duration(0.2).sleep();
    }
    // Increment the default iterator
    i++;
    // There are only 9 available data rates, if no connection
    // made yet possibly a hardware malfunction?
    if (i > 8) {
      break;
    }
  }

  // Now we verify connection (Should be good if we made it this far)
  if (vs.verifySensorConnectivity()) {
    ROS_INFO("Device connection established");
  } else {
    ROS_ERROR("No device communication");
    ROS_WARN("Please input a valid baud rate. Valid are:");
    ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
    ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
  }
  // Query the sensor's model number.
  string mn = vs.readModelNumber();
  string fv = vs.readFirmwareVersion();
  uint32_t hv = vs.readHardwareRevision();
  uint32_t sn = vs.readSerialNumber();
  ROS_INFO("Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());
  ROS_INFO("Hardware Revision : %d, Serial Number : %d", hv, sn);

  // calculate the least common multiple of the two rate and assure it is a
  // valid package rate, also calculate the imu and output strides
  int package_rate = 0;
  for (int allowed_rate : {1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200, 0}) {
    package_rate = allowed_rate;
    if ((package_rate % async_output_rate) == 0 && (package_rate % imu_output_rate) == 0) break;
  }
  ROS_ASSERT_MSG(
    package_rate,
    "imu_output_rate (%d) or async_output_rate (%d) is not in 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, "
    "200 Hz",
    imu_output_rate, async_output_rate);
  user_data.imu_stride = package_rate / imu_output_rate;
  user_data.output_stride = package_rate / async_output_rate;
  ROS_INFO("Package Receive Rate: %d Hz", package_rate);
  ROS_INFO("General Publish Rate: %d Hz", async_output_rate);
  ROS_INFO("IMU Publish Rate: %d Hz", imu_output_rate);

  // Set the device info for passing to the packet callback function
  user_data.device_family = vs.determineDeviceFamily();

  // Make sure no generic async output is registered
  vs.writeAsyncDataOutputType(VNOFF);

  // Configure binary output message
  BinaryOutputRegister bor(
    ASYNCMODE_PORT1,
    SensorImuRate / package_rate,  // update rate [ms]
    COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE |
      COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES |
      (user_data.adjust_ros_timestamp ? COMMONGROUP_TIMESTARTUP : 0),
    TIMEGROUP_NONE | TIMEGROUP_GPSTOW | TIMEGROUP_GPSWEEK | TIMEGROUP_TIMEUTC, IMUGROUP_NONE,
    GPSGROUP_NONE,
    ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
    INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF |
      INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU,
    GPSGROUP_NONE);

  // An empty output register for disabling output 2 and 3 if previously set
  BinaryOutputRegister bor_none(
    0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
    INSGROUP_NONE, GPSGROUP_NONE);

  vs.writeBinaryOutput1(bor);
  vs.writeBinaryOutput2(bor_none);
  vs.writeBinaryOutput3(bor_none);

  // Register async callback function
  vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

  // You spin me right round, baby
  // Right round like a record, baby
  // Right round round round
  while (ros::ok()) {
    ros::spin();  // Need to make sure we disconnect properly. Check if all ok.
  }

  // Node has been terminated
  vs.unregisterAsyncPacketReceivedHandler();
  ros::Duration(0.5).sleep();
  ROS_INFO("Unregisted the Packet Received Handler");
  vs.disconnect();
  ros::Duration(0.5).sleep();
  ROS_INFO("%s is disconnected successfully", mn.c_str());
  return 0;
}

//Helper function to create IMU message
void fill_imu_message(
  sensor_msgs::Imu & msgIMU, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgIMU.header.stamp = time;
  msgIMU.header.frame_id = user_data->frame_id;

  if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration()) {
    vec4f q = cd.quaternion();
    vec3f ar = cd.angularRate();
    vec3f al = cd.acceleration();

    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      msgIMU.orientation_covariance[0] =
        pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Roll
      msgIMU.orientation_covariance[4] =
        pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians Pitch
      msgIMU.orientation_covariance[8] =
        pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Yaw
    }

    //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
    if (user_data->tf_ned_to_enu) {
      // If we want the orientation to be based on the reference label on the imu
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      geometry_msgs::Quaternion quat_msg;

      if (user_data->frame_based_enu) {
        // Create a rotation from NED -> ENU
        tf2::Quaternion q_rotate;
        q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
        // Apply the NED to ENU rotation such that the coordinate frame matches
        tf2_quat = q_rotate * tf2_quat;
        quat_msg = tf2::toMsg(tf2_quat);

        // Since everything is in the normal frame, no flipping required
        msgIMU.angular_velocity.x = ar[0];
        msgIMU.angular_velocity.y = ar[1];
        msgIMU.angular_velocity.z = ar[2];

        msgIMU.linear_acceleration.x = al[0];
        msgIMU.linear_acceleration.y = al[1];
        msgIMU.linear_acceleration.z = al[2];
      } else {
        // put into ENU - swap X/Y, invert Z
        quat_msg.x = q[1];
        quat_msg.y = q[0];
        quat_msg.z = -q[2];
        quat_msg.w = q[3];

        // Flip x and y then invert z
        msgIMU.angular_velocity.x = ar[1];
        msgIMU.angular_velocity.y = ar[0];
        msgIMU.angular_velocity.z = -ar[2];
        // Flip x and y then invert z
        msgIMU.linear_acceleration.x = al[1];
        msgIMU.linear_acceleration.y = al[0];
        msgIMU.linear_acceleration.z = -al[2];

        if (cd.hasAttitudeUncertainty()) {
          vec3f orientationStdDev = cd.attitudeUncertainty();
          msgIMU.orientation_covariance[0] =
            pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians pitch
          msgIMU.orientation_covariance[4] =
            pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Roll
          msgIMU.orientation_covariance[8] =
            pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Yaw
        }
      }

      msgIMU.orientation = quat_msg;
    } else {
      msgIMU.orientation.x = q[0];
      msgIMU.orientation.y = q[1];
      msgIMU.orientation.z = q[2];
      msgIMU.orientation.w = q[3];

      msgIMU.angular_velocity.x = ar[0];
      msgIMU.angular_velocity.y = ar[1];
      msgIMU.angular_velocity.z = ar[2];
      msgIMU.linear_acceleration.x = al[0];
      msgIMU.linear_acceleration.y = al[1];
      msgIMU.linear_acceleration.z = al[2];
    }
    // Covariances pulled from parameters
    msgIMU.angular_velocity_covariance = user_data->angular_vel_covariance;
    msgIMU.linear_acceleration_covariance = user_data->linear_accel_covariance;
  }
}

//Helper function to create magnetic field message
void fill_mag_message(
  sensor_msgs::MagneticField & msgMag, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgMag.header.stamp = time;
  msgMag.header.frame_id = user_data->frame_id;

  // Magnetic Field
  if (cd.hasMagnetic()) {
    vec3f mag = cd.magnetic();
    msgMag.magnetic_field.x = mag[0];
    msgMag.magnetic_field.y = mag[1];
    msgMag.magnetic_field.z = mag[2];
  }
}

//Helper function to create gps message
void fill_gps_message(
  sensor_msgs::NavSatFix & msgGPS, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgGPS.header.stamp = time;
  msgGPS.header.frame_id = user_data->frame_id;

  if (cd.hasPositionEstimatedLla()) {
    vec3d lla = cd.positionEstimatedLla();

    msgGPS.latitude = lla[0];
    msgGPS.longitude = lla[1];
    msgGPS.altitude = lla[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      msgGPS.position_covariance[0] = posVariance;  // East position variance
      msgGPS.position_covariance[4] = posVariance;  // North position vaciance
      msgGPS.position_covariance[8] = posVariance;  // Up position variance

      // mark gps fix as not available if the outputted standard deviation is 0
      if (cd.positionUncertaintyEstimated() != 0.0) {
        // Position available
        msgGPS.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      } else {
        // position not detected
        msgGPS.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      }

      // add the type of covariance to the gps message
      msgGPS.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      msgGPS.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }
}

//Helper function to create odometry message
void fill_odom_message(
  nav_msgs::Odometry & msgOdom, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgOdom.header.stamp = time;
  msgOdom.child_frame_id = user_data->frame_id;
  msgOdom.header.frame_id = user_data->map_frame_id;

  if (cd.hasPositionEstimatedEcef()) {
    // add position as earth fixed frame
    vec3d pos = cd.positionEstimatedEcef();

    if (!user_data->initial_position_set) {
      ROS_INFO("Set initial position to %f %f %f", pos[0], pos[1], pos[2]);
      user_data->initial_position_set = true;
      user_data->initial_position.x = pos[0];
      user_data->initial_position.y = pos[1];
      user_data->initial_position.z = pos[2];
    }

    msgOdom.pose.pose.position.x = pos[0] - user_data->initial_position[0];
    msgOdom.pose.pose.position.y = pos[1] - user_data->initial_position[1];
    msgOdom.pose.pose.position.z = pos[2] - user_data->initial_position[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      msgOdom.pose.covariance[0] = posVariance;   // x-axis position variance
      msgOdom.pose.covariance[7] = posVariance;   // y-axis position vaciance
      msgOdom.pose.covariance[14] = posVariance;  // z-axis position variance
    }
  }

  if (cd.hasQuaternion()) {
    vec4f q = cd.quaternion();

    if (!user_data->tf_ned_to_enu) {
      // output in NED frame
      msgOdom.pose.pose.orientation.x = q[0];
      msgOdom.pose.pose.orientation.y = q[1];
      msgOdom.pose.pose.orientation.z = q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    } else if (user_data->tf_ned_to_enu && user_data->frame_based_enu) {
      // standard conversion from NED to ENU frame
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // Create a rotation from NED -> ENU
      tf2::Quaternion q_rotate;
      q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
      // Apply the NED to ENU rotation such that the coordinate frame matches
      tf2_quat = q_rotate * tf2_quat;
      msgOdom.pose.pose.orientation = tf2::toMsg(tf2_quat);
    } else if (user_data->tf_ned_to_enu && !user_data->frame_based_enu) {
      // alternative method for conversion to ENU frame (leads to another result)
      // put into ENU - swap X/Y, invert Z
      msgOdom.pose.pose.orientation.x = q[1];
      msgOdom.pose.pose.orientation.y = q[0];
      msgOdom.pose.pose.orientation.z = -q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      // convert the standard deviation values from all three axis from degrees to radiant and calculate the variances from these (squared), which are assigned to the covariance matrix.
      if (!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
        // standard assignment of variance values for NED frame and conversion to ENU frame by rotation
        msgOdom.pose.covariance[21] = pow(orientationStdDev[0] * M_PI / 180, 2);  // roll variance
        msgOdom.pose.covariance[28] = pow(orientationStdDev[1] * M_PI / 180, 2);  // pitch variance
        msgOdom.pose.covariance[35] = pow(orientationStdDev[2] * M_PI / 180, 2);  // yaw variance
      } else {
        // variance assignment for conversion by swapping and inverting (not frame_based_enu)

        // TODO not supported yet
      }
    }
  }

  // Add the velocity in the body frame (frame_id) to the message
  if (cd.hasVelocityEstimatedBody()) {
    vec3f vel = cd.velocityEstimatedBody();

    if (!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
      // standard assignment of values for NED frame and conversion to ENU frame by rotation
      msgOdom.twist.twist.linear.x = vel[0];
      msgOdom.twist.twist.linear.y = vel[1];
      msgOdom.twist.twist.linear.z = vel[2];
    } else {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      msgOdom.twist.twist.linear.x = vel[1];
      msgOdom.twist.twist.linear.y = vel[0];
      msgOdom.twist.twist.linear.z = -vel[2];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasVelocityUncertaintyEstimated()) {
      double velVariance = pow(cd.velocityUncertaintyEstimated(), 2);
      msgOdom.twist.covariance[0] = velVariance;   // x-axis velocity variance
      msgOdom.twist.covariance[7] = velVariance;   // y-axis velocity vaciance
      msgOdom.twist.covariance[14] = velVariance;  // z-axis velocity variance

      // set velocity variances to a high value if no data is available (this is the case at startup during INS is initializing)
      if (
        msgOdom.twist.twist.linear.x == 0 && msgOdom.twist.twist.linear.y == 0 &&
        msgOdom.twist.twist.linear.z == 0 && msgOdom.twist.covariance[0] == 0 &&
        msgOdom.twist.covariance[7] == 0 && msgOdom.twist.covariance[14] == 0) {
        msgOdom.twist.covariance[0] = 200;
        msgOdom.twist.covariance[7] = 200;
        msgOdom.twist.covariance[15] = 200;
      }
    }
  }

  if (cd.hasAngularRate()) {
    vec3f ar = cd.angularRate();

    if (!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
      // standard assignment of values for NED frame and conversion to ENU frame by rotation
      msgOdom.twist.twist.angular.x = ar[0];
      msgOdom.twist.twist.angular.y = ar[1];
      msgOdom.twist.twist.angular.z = ar[2];
    } else {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      msgOdom.twist.twist.angular.x = ar[1];
      msgOdom.twist.twist.angular.y = ar[0];
      msgOdom.twist.twist.angular.z = -ar[2];
    }

    // add covariance matrix of the measured angular rate to odom message.
    // go through matrix rows
    for (int row = 0; row < 3; row++) {
      // go through matrix columns
      for (int col = 0; col < 3; col++) {
        // Target matrix has 6 rows and 6 columns, source matrix has 3 rows and 3 columns. The covariance values are put into the fields (3, 3) to (5, 5) of the destination matrix.
        msgOdom.twist.covariance[(row + 3) * 6 + (col + 3)] =
          user_data->angular_vel_covariance[row * 3 + col];
      }
    }
  }
}

//Helper function to create temperature message
void fill_temp_message(
  sensor_msgs::Temperature & msgTemp, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgTemp.header.stamp = time;
  msgTemp.header.frame_id = user_data->frame_id;
  if (cd.hasTemperature()) {
    float temp = cd.temperature();
    msgTemp.temperature = temp;
  }
}

//Helper function to create pressure message
void fill_pres_message(
  sensor_msgs::FluidPressure & msgPres, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgPres.header.stamp = time;
  msgPres.header.frame_id = user_data->frame_id;
  if (cd.hasPressure()) {
    float pres = cd.pressure();
    msgPres.fluid_pressure = pres;
  }
}

//Helper function to create ins message
void fill_ins_message(
  vectornav::Ins & msgINS, vn::sensors::CompositeData & cd, ros::Time & time, UserData * user_data)
{
  msgINS.header.stamp = time;
  msgINS.header.frame_id = user_data->frame_id;

  if (cd.hasInsStatus()) {
    InsStatus insStatus = cd.insStatus();
    msgINS.insStatus = static_cast<uint16_t>(insStatus);
  }

  if (cd.hasTow()) {
    msgINS.time = cd.tow();
  }

  if (cd.hasWeek()) {
    msgINS.week = cd.week();
  }

  if (cd.hasTimeUtc()) {
    TimeUtc utcTime = cd.timeUtc();
    char * utcTimeBytes = reinterpret_cast<char *>(&utcTime);
    //msgINS.utcTime bytes are in Little Endian Byte Order
    std::memcpy(&msgINS.utcTime, utcTimeBytes, 8);
  }

  if (cd.hasYawPitchRoll()) {
    vec3f rpy = cd.yawPitchRoll();
    msgINS.yaw = rpy[0];
    msgINS.pitch = rpy[1];
    msgINS.roll = rpy[2];
  }

  if (cd.hasPositionEstimatedLla()) {
    vec3d lla = cd.positionEstimatedLla();
    msgINS.latitude = lla[0];
    msgINS.longitude = lla[1];
    msgINS.altitude = lla[2];
  }

  if (cd.hasVelocityEstimatedNed()) {
    vec3f nedVel = cd.velocityEstimatedNed();
    msgINS.nedVelX = nedVel[0];
    msgINS.nedVelY = nedVel[1];
    msgINS.nedVelZ = nedVel[2];
  }

  if (cd.hasAttitudeUncertainty()) {
    vec3f attUncertainty = cd.attitudeUncertainty();
    msgINS.attUncertainty[0] = attUncertainty[0];
    msgINS.attUncertainty[1] = attUncertainty[1];
    msgINS.attUncertainty[2] = attUncertainty[2];
  }

  if (cd.hasPositionUncertaintyEstimated()) {
    msgINS.posUncertainty = cd.positionUncertaintyEstimated();
  }

  if (cd.hasVelocityUncertaintyEstimated()) {
    msgINS.velUncertainty = cd.velocityUncertaintyEstimated();
  }
}

static ros::Time get_time_stamp(
  vn::sensors::CompositeData & cd, UserData * user_data, const ros::Time & ros_time)
{
  if (!cd.hasTimeStartup() || !user_data->adjust_ros_timestamp) {
    return (ros_time);  // don't adjust timestamp
  }
  const double sensor_time = cd.timeStartup() * 1e-9;  // time in seconds
  if (user_data->average_time_difference == 0) {       // first call
    user_data->ros_start_time = ros_time;
    user_data->average_time_difference = static_cast<double>(-sensor_time);
  }
  // difference between node startup and current ROS time
  const double ros_dt = (ros_time - user_data->ros_start_time).toSec();
  // difference between elapsed ROS time and time since sensor startup
  const double dt = ros_dt - sensor_time;
  // compute exponential moving average
  const double alpha = 0.001;  // average over rougly 1000 samples
  user_data->average_time_difference =
    user_data->average_time_difference * (1.0 - alpha) + alpha * dt;

  // adjust sensor time by average difference to ROS time
  const ros::Time adj_time =
    user_data->ros_start_time + ros::Duration(user_data->average_time_difference + sensor_time);
  return (adj_time);
}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index)
{
  // package counter to calculate strides
  static unsigned long long pkg_count = 0;

  // evaluate time first, to have it as close to the measurement time as possible
  const ros::Time ros_time = ros::Time::now();

  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  UserData * user_data = static_cast<UserData *>(userData);
  ros::Time time = get_time_stamp(cd, user_data, ros_time);

  // IMU
  if ((pkg_count % user_data->imu_stride) == 0 && pubIMU.getNumSubscribers() > 0) {
    sensor_msgs::Imu msgIMU;
    fill_imu_message(msgIMU, cd, time, user_data);
    pubIMU.publish(msgIMU);
  }

  if ((pkg_count % user_data->output_stride) == 0) {
    // Magnetic Field
    if (pubMag.getNumSubscribers() > 0) {
      sensor_msgs::MagneticField msgMag;
      fill_mag_message(msgMag, cd, time, user_data);
      pubMag.publish(msgMag);
    }

    // Temperature
    if (pubTemp.getNumSubscribers() > 0) {
      sensor_msgs::Temperature msgTemp;
      fill_temp_message(msgTemp, cd, time, user_data);
      pubTemp.publish(msgTemp);
    }

    // Barometer
    if (pubPres.getNumSubscribers() > 0) {
      sensor_msgs::FluidPressure msgPres;
      fill_pres_message(msgPres, cd, time, user_data);
      pubPres.publish(msgPres);
    }

    // GPS
    if (
      user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
      pubGPS.getNumSubscribers() > 0) {
      sensor_msgs::NavSatFix msgGPS;
      fill_gps_message(msgGPS, cd, time, user_data);
      pubGPS.publish(msgGPS);
    }

    // Odometry
    if (
      user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
      pubOdom.getNumSubscribers() > 0) {
      nav_msgs::Odometry msgOdom;
      fill_odom_message(msgOdom, cd, time, user_data);
      pubOdom.publish(msgOdom);
    }

    // INS
    if (
      user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
      pubIns.getNumSubscribers() > 0) {
      vectornav::Ins msgINS;
      fill_ins_message(msgINS, cd, time, user_data);
      pubIns.publish(msgINS);
    }
  }
  pkg_count += 1;
}
