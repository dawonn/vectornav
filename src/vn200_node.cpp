/*
 * MIT License (MIT)
 *
 * Copyright (c) 2013 Dereck Wonnacott <dereck@gmail.com>
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


#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>     /* exit, EXIT_FAILURE */

#include <vn/sensors.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <signal.h>
#include <time.h>
// Message Types
#include <vectornav/utc_time.h>
#include <vectornav/gps.h>
#include <vectornav/ins.h>
#include <vectornav/sensors.h>

#include <ros/xmlrpc_manager.h>

#include <iostream>

using namespace vn::protocol::uart;
using namespace vn::sensors;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;


// Params
std::string imu_frame_id, gps_frame_id;

// Publishers
ros::Publisher pub_ins;
ros::Publisher pub_gps;
ros::Publisher pub_sensors;
ros::Publisher pub_sync_in;

// Device
vn::sensors::VnSensor vn200;

int ins_seq           = 0;
int imu_seq           = 0;
int gps_seq           = 0;
int msg_cnt           = 0;
int last_group_number = 0;


std::string port;
char vn_error_msg[100];

struct utc_time_struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
};

struct ins_binary_data_struct 
{
    uint64_t gps_time;
    uint64_t sync_in_time;
    vn::math::vec3f ypr;
    vn::math::vec3d lla;
    vn::math::vec3f vel_ned;
    uint16_t ins_status;
    uint32_t sync_in_count;
    utc_time_struct utc_time;
    vn::math::vec3f ypru;
    float pos_sigma;
    float vel_sigma;
};

ins_binary_data_struct ins_binary_data;

struct gps_binary_data_struct
{
    utc_time_struct utc_time;
    uint64_t tow;
    uint16_t week;
    uint8_t  num_sats;
    uint8_t  fix;
    vn::math::vec3d lla;
    vn::math::vec3f vel_ned;
    vn::math::vec3f posu;
    float vel_sigma;
    float time_sigma;
};

struct gps_binary_data_struct gps_binary_data;

struct imu_binary_data_struct
{
    uint64_t gps_time;
    vn::math::vec3f accel;
    vn::math::vec3f angular_rate;
};

struct imu_binary_data_struct imu_binary_data;

int ins_msg_count = 0;
int gps_msg_count = 0;
int imu_msg_count = 0;

const unsigned raw_imu_max_rate = 800;

const unsigned gps_group_signature = BINARYGROUP_GPS;
const unsigned ins_group_signature = BINARYGROUP_COMMON | BINARYGROUP_ATTITUDE
                                     | BINARYGROUP_TIME | BINARYGROUP_INS;
const unsigned imu_group_signature = BINARYGROUP_COMMON;

void publish_gps_data()
{
    gps_seq++;
    ros::Time timestamp =  ros::Time::now(); 

    if (pub_gps.getNumSubscribers() > 0)
    {
        vectornav::gps msg_gps;
        msg_gps.header.seq      = gps_seq;
        msg_gps.header.stamp    = timestamp;
        msg_gps.header.frame_id = "gps";

        msg_gps.UtcTime.year = gps_binary_data.utc_time.year;
        msg_gps.UtcTime.month = gps_binary_data.utc_time.month;
        msg_gps.UtcTime.day = gps_binary_data.utc_time.day;
        msg_gps.UtcTime.hour = gps_binary_data.utc_time.hour;
        msg_gps.UtcTime.minute = gps_binary_data.utc_time.minute;
        msg_gps.UtcTime.second = gps_binary_data.utc_time.second;
        msg_gps.UtcTime.millisecond = gps_binary_data.utc_time.millisecond;

        msg_gps.Time    = (double)gps_binary_data.tow*1E-9;
        msg_gps.Week    = gps_binary_data.week;
        msg_gps.GpsFix  = gps_binary_data.fix;
        msg_gps.NumSats = gps_binary_data.num_sats;

        msg_gps.LLA.x = gps_binary_data.lla[0];
        msg_gps.LLA.y = gps_binary_data.lla[1];
        msg_gps.LLA.z = gps_binary_data.lla[2];

        msg_gps.NedVel.x = gps_binary_data.vel_ned[0];
        msg_gps.NedVel.y = gps_binary_data.vel_ned[1];
        msg_gps.NedVel.z = gps_binary_data.vel_ned[2];

        msg_gps.NedAcc.x = gps_binary_data.posu[0];
        msg_gps.NedAcc.y = gps_binary_data.posu[1];
        msg_gps.NedAcc.z = gps_binary_data.posu[2];

        msg_gps.SpeedAcc = gps_binary_data.vel_sigma;
        msg_gps.TimeAcc  = gps_binary_data.time_sigma;

        pub_gps.publish(msg_gps);
    }
}

void publish_ins_data()
{
    ins_seq++;
    ros::Time timestamp =  ros::Time::now(); 

    if (pub_ins.getNumSubscribers() > 0)
    {
        vectornav::ins msg_ins;
        msg_ins.header.seq      = ins_seq;
        msg_ins.header.stamp    = timestamp;
        msg_ins.header.frame_id = "ins";

        msg_ins.Time    = (double)ins_binary_data.gps_time*1E-9;
        msg_ins.Week    = gps_binary_data.week;
        msg_ins.Status  = ins_binary_data.ins_status;

        msg_ins.RPY.x = ins_binary_data.ypr[2]; // Intentional re-ordering
        msg_ins.RPY.y = ins_binary_data.ypr[1];
        msg_ins.RPY.z = ins_binary_data.ypr[0];

        msg_ins.LLA.x = ins_binary_data.lla[0];
        msg_ins.LLA.y = ins_binary_data.lla[1];
        msg_ins.LLA.z = ins_binary_data.lla[2];

        msg_ins.NedVel.x = ins_binary_data.vel_ned[0];
        msg_ins.NedVel.y = ins_binary_data.vel_ned[1];
        msg_ins.NedVel.z = ins_binary_data.vel_ned[2];

        msg_ins.RollUncertainty = ins_binary_data.ypru[2];
        msg_ins.PitchUncertainty = ins_binary_data.ypru[1];
        msg_ins.YawUncertainty = ins_binary_data.ypru[0];

        msg_ins.PosUncertainty  = ins_binary_data.pos_sigma;
        msg_ins.VelUncertainty  = ins_binary_data.vel_sigma;

        std::tm utc_formatted_time;
        // vectornav decided to start from 2000, whereas struct tm measures
        // from 1900.
        utc_formatted_time.tm_year  = ins_binary_data.utc_time.year + 100;

        // vectornav encodes months counting from 1, whereas struct tm
        // counts from 0.
        utc_formatted_time.tm_mon   = ins_binary_data.utc_time.month - 1;
        utc_formatted_time.tm_mday  = ins_binary_data.utc_time.day;
        utc_formatted_time.tm_hour  = ins_binary_data.utc_time.hour;
        utc_formatted_time.tm_min   = ins_binary_data.utc_time.minute;
        utc_formatted_time.tm_sec   = ins_binary_data.utc_time.second;
        utc_formatted_time.tm_wday  = 0; // Ignored by mktime.
        utc_formatted_time.tm_yday  = 0; // Ignored by mktime.
        utc_formatted_time.tm_isdst = 0; // UTC never sets daylight savings.

        std::time_t epoch_time = timegm(&utc_formatted_time);
        double epoch_time_s = static_cast<double>(epoch_time) +
          static_cast<double>(ins_binary_data.utc_time.millisecond)*1e-3;
        double sync_in_delta = static_cast<double>(ins_binary_data.sync_in_time)*1e-9;
        msg_ins.SyncInTime 	    = epoch_time_s - sync_in_delta;
        msg_ins.SyncInCount     = ins_binary_data.sync_in_count;

        pub_ins.publish(msg_ins);
    } 
}

void publish_imu_data()
{
    ++imu_seq;
    ros::Time timestamp =  ros::Time::now(); 
    // IMU Data
    if (pub_sensors.getNumSubscribers() > 0) {
        vectornav::sensors msg_sensors;
        msg_sensors.header.seq      = imu_seq;
        msg_sensors.header.stamp    = timestamp;
        msg_sensors.header.frame_id = "imu";
        msg_sensors.gps_time 	    = (double)imu_binary_data.gps_time*1E-9;
        msg_sensors.Accel.x = imu_binary_data.accel[0];
        msg_sensors.Accel.y = imu_binary_data.accel[1];
        msg_sensors.Accel.z = imu_binary_data.accel[2];

        msg_sensors.Gyro.x = imu_binary_data.angular_rate[0];
        msg_sensors.Gyro.y = imu_binary_data.angular_rate[1];
        msg_sensors.Gyro.z = imu_binary_data.angular_rate[2];

        pub_sensors.publish(msg_sensors);
    }
}

void binaryMessageReceived(void * user_data, Packet & p, size_t index)
{
    std::string raw_data;
    if (p.type() == Packet::TYPE_BINARY && p.isValid()) {
        switch (p.groups()) {
        case gps_group_signature:
            ++gps_msg_count;
            gps_binary_data.utc_time.year =        p.extractUint8();
            gps_binary_data.utc_time.month =       p.extractUint8();
            gps_binary_data.utc_time.day =         p.extractUint8();
            gps_binary_data.utc_time.hour =        p.extractUint8();
            gps_binary_data.utc_time.minute =      p.extractUint8();
            gps_binary_data.utc_time.second =      p.extractUint8();
            gps_binary_data.utc_time.millisecond = p.extractUint16();

            gps_binary_data.tow =        p.extractUint64();
            gps_binary_data.week =       p.extractUint16();
            gps_binary_data.num_sats =   p.extractUint8();
            gps_binary_data.fix =        p.extractUint8();
            gps_binary_data.lla =        p.extractVec3d();
            gps_binary_data.vel_ned =    p.extractVec3f();
            gps_binary_data.posu =       p.extractVec3f();
            gps_binary_data.vel_sigma =  p.extractFloat();
            gps_binary_data.time_sigma = p.extractFloat();

            publish_gps_data();
            if (remainder(gps_msg_count, 16) == 0) {
                gps_msg_count = 0;
                ROS_INFO_STREAM("TOW: " << gps_binary_data.tow*1E-9 << " NumSats: " << (int)gps_binary_data.num_sats);
            }
            break;
        case ins_group_signature:
            ++ins_msg_count;
            ins_binary_data.gps_time =      p.extractUint64();
            ins_binary_data.sync_in_time =  p.extractUint64();
            ins_binary_data.ypr =           p.extractVec3f();
            ins_binary_data.lla =           p.extractVec3d();
            ins_binary_data.vel_ned =       p.extractVec3f();
            ins_binary_data.ins_status =    p.extractUint16();
            ins_binary_data.sync_in_count = p.extractUint32();

            ins_binary_data.utc_time.year =        p.extractUint8();
            ins_binary_data.utc_time.month =       p.extractUint8();
            ins_binary_data.utc_time.day =         p.extractUint8();
            ins_binary_data.utc_time.hour =        p.extractUint8();
            ins_binary_data.utc_time.minute =      p.extractUint8();
            ins_binary_data.utc_time.second =      p.extractUint8();
            ins_binary_data.utc_time.millisecond = p.extractUint16();

            ins_binary_data.ypru =          p.extractVec3f();
            ins_binary_data.pos_sigma =     p.extractFloat();
            ins_binary_data.vel_sigma =     p.extractFloat();

            publish_ins_data();

            if (remainder(ins_msg_count, 100) == 0) {
                ins_msg_count = 0;
                ROS_INFO_STREAM("INS_Time: " << ins_binary_data.gps_time*1E-9 << " Yaw: " <<
                        ins_binary_data.ypr[0] << " Pitch: " << ins_binary_data.ypr[1] <<
                        " Roll: " << ins_binary_data.ypr[2] << " INS_Status: " <<
                        ins_binary_data.ins_status << " latitude: " << ins_binary_data.lla[0] <<
                        " longitude: " << ins_binary_data.lla[1] << " altitude: " << ins_binary_data.lla[2]);
            }
            break;
        case imu_group_signature:
            ++imu_msg_count;
            imu_binary_data.gps_time =     p.extractUint64();
            imu_binary_data.accel =        p.extractVec3f();
            imu_binary_data.angular_rate = p.extractVec3f();

            publish_imu_data();
            if (remainder(imu_msg_count, 500) == 0) {
                imu_msg_count = 0;
                ROS_INFO_STREAM("IMU_Time: " << imu_binary_data.gps_time*1E-9 << " rotr_x: " <<
                        imu_binary_data.angular_rate[0] << " rotr_y: " << imu_binary_data.angular_rate[1] <<
                        " rotr_z: " << imu_binary_data.angular_rate[2] << " accel_x: " <<
                        imu_binary_data.accel[0] << " accel_y: " << imu_binary_data.accel[1] <<
                        " accel_z: " << imu_binary_data.accel[2]);
            }
            break;
        default:
            ROS_WARN("Received unknown group signature from vectornav");
        }
    } else {
        ROS_WARN("Received invalid packet from vectornav.");
        // Ignore non-binary packets for now.
    }
}

bool dividesEvenly(unsigned numerator, unsigned denominator)
{
    unsigned dividend = numerator / denominator;
    return numerator == denominator * dividend;
}

void mySigintHandler(int sig)
{
    g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;

    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        num_params = params.size();
    }
    if (num_params > 1) {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

/////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    // Initialize ROS;
    ros::init(argc, argv, "vectornav", ros::init_options::NoSigintHandler);
    ros::NodeHandle n; 
    ros::NodeHandle n_("~");

    // Read Parameters
    int baud, poll_rate_ins, poll_rate_gps, poll_rate_imu, async_output_type, async_output_rate,
        binary_data_output_port, binary_gps_data_rate, binary_ins_data_rate,
        binary_imu_data_rate;

    int retry_cnt = 0;

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
    signal(SIGINT, mySigintHandler);

    n_.param<std::string>("serial_port" , port     , "/dev/ttyUSB0");
    n_.param<int>(        "serial_baud" , baud     , 115200);
    n_.param<int>(        "poll_rate_gps"   , poll_rate_gps, 5);
    n_.param<int>(        "poll_rate_ins"   , poll_rate_ins, 20);
    n_.param<int>(        "poll_rate_imu"   , poll_rate_imu, 100);

    n_.param<std::string>("imu/frame_id", imu_frame_id, "LLA");
    n_.param<std::string>("gps/frame_id", gps_frame_id, "LLA");

    // Type: 0 None, 19 IMU, 20 GPS, 22 INS
    n_.param<int>(        "async_output_type"  , async_output_type, 0);
    n_.param<int>(        "async_output_rate"  , async_output_rate, 50); 
    
    n_.param<int>(        "binary_data_output_port"  , binary_data_output_port, 1); 
    n_.param<int>(        "binary_gps_data_output_rate"  , binary_gps_data_rate, 4); 
    n_.param<int>(        "binary_ins_data_output_rate"  , binary_ins_data_rate, 20); 
    n_.param<int>(        "binary_imu_data_output_rate"  , binary_imu_data_rate, 100); 

    // Validate the rate inputs.
    if (binary_gps_data_rate < 1 || binary_gps_data_rate > raw_imu_max_rate) {
        ROS_FATAL_STREAM("binary_gps_data_output_rate of "
          << binary_gps_data_rate << " is out of range (1 - "
          << raw_imu_max_rate << ")");
        exit(EXIT_FAILURE);
    } else if (!dividesEvenly(raw_imu_max_rate, binary_gps_data_rate)) {
        ROS_FATAL_STREAM("binary_gps_data_rate does not evenly divide "
          << raw_imu_max_rate);
        exit(EXIT_FAILURE);
    }

    if (binary_ins_data_rate < 1 || binary_ins_data_rate > raw_imu_max_rate) {
        ROS_FATAL_STREAM("binary_ins_data_rate of "
          << binary_ins_data_rate << " is out of range (1 - "
          << raw_imu_max_rate << ")");
        exit(EXIT_FAILURE);
    } else if (!dividesEvenly(raw_imu_max_rate, binary_ins_data_rate)) {
        ROS_FATAL_STREAM("binary_ins_data_rate does not evenly divide "
          << raw_imu_max_rate);
        exit(EXIT_FAILURE);
    }

    if (binary_imu_data_rate < 1 || binary_imu_data_rate > raw_imu_max_rate) {
        ROS_FATAL_STREAM("binary_imu_data_rate of "
          << binary_imu_data_rate << " is out of range (1 - "
          << raw_imu_max_rate << ")");
        exit(EXIT_FAILURE);
    } else if (!dividesEvenly(raw_imu_max_rate, binary_imu_data_rate)) {
        ROS_FATAL_STREAM("binary_imu_data_rate does not evenly divide "
          << raw_imu_max_rate);
        exit(EXIT_FAILURE);
    }

    AsyncMode binary_data_output_mode =
      static_cast<AsyncMode>(binary_data_output_port);

    // Initialize Publishers
    pub_ins     = n_.advertise<vectornav::ins>    ("ins", 1000);
    pub_gps     = n_.advertise<vectornav::gps>    ("gps", 1000);
    pub_sensors = n_.advertise<vectornav::sensors>("imu", 1000);

    // Initialize VectorNav
    //VN_ERROR_CODE vn_retval;
    ROS_INFO("Initializing vn200. Port:%s Baud:%d\n", port.c_str(), baud);

    try {
        vn200.connect(port, baud);
    } catch (...) {
        ROS_FATAL("Could not conenct to vn200 on port:%s @ Baud:%d;"
                "Did you add your user to the 'dialout' group in /etc/group?", 
                port.c_str(), 
                baud); 
        exit(EXIT_FAILURE);
    }

    GpsGroup gps_gps_group = GPSGROUP_UTC | GPSGROUP_TOW | GPSGROUP_WEEK
        | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELNED
        | GPSGROUP_POSU | GPSGROUP_VELU | GPSGROUP_TIMEU;

    BinaryOutputRegister gps_log_reg(
        binary_data_output_mode,
        raw_imu_max_rate / binary_gps_data_rate,
        COMMONGROUP_NONE,
        TIMEGROUP_NONE,
        IMUGROUP_NONE,
        gps_gps_group,
        ATTITUDEGROUP_NONE,
        INSGROUP_NONE);


    CommonGroup ins_common_group = COMMONGROUP_TIMEGPS | COMMONGROUP_TIMESYNCIN
        | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_POSITION
        | COMMONGROUP_VELOCITY | COMMONGROUP_INSSTATUS | COMMONGROUP_SYNCINCNT;

    TimeGroup ins_time_group = TIMEGROUP_TIMEUTC;

    AttitudeGroup ins_attitude_group = ATTITUDEGROUP_YPRU;

    InsGroup ins_ins_group = INSGROUP_POSU | INSGROUP_VELU;

    BinaryOutputRegister ins_log_reg(
        binary_data_output_mode,
        raw_imu_max_rate / binary_ins_data_rate,
        ins_common_group,
        ins_time_group,
        IMUGROUP_NONE,
        GPSGROUP_NONE,
        ins_attitude_group,
        ins_ins_group);


    CommonGroup imu_common_group = COMMONGROUP_TIMEGPS | COMMONGROUP_ACCEL
        | COMMONGROUP_ANGULARRATE;

    BinaryOutputRegister imu_log_reg(
        binary_data_output_mode,
        raw_imu_max_rate / binary_imu_data_rate,
        imu_common_group,
        TIMEGROUP_NONE,
        IMUGROUP_NONE,
        GPSGROUP_NONE,
        ATTITUDEGROUP_NONE,
        INSGROUP_NONE);

    vn200.writeBinaryOutput1(gps_log_reg);
    vn200.writeBinaryOutput2(ins_log_reg);
    vn200.writeBinaryOutput3(imu_log_reg);

    ROS_INFO("About to set SynchronizationControl");

    vn::sensors::SynchronizationControlRegister sync_control(
        SYNCINMODE_COUNT,
        SYNCINEDGE_RISING,
        0, // sync in skip factor
        SYNCOUTMODE_GPSPPS,
        SYNCOUTPOLARITY_NEGATIVE,
        0, // sync out skip factor
        1000000); // a millisecond should be fine.

    vn200.writeSynchronizationControl(sync_control);

    vn::math::vec3f position;
    position[0] = 0.0;
    position[1] = -0.039;
    position[2] = 0.0;

    vn200.writeGpsAntennaOffset(position);
    vn200.registerAsyncPacketReceivedHandler(NULL, binaryMessageReceived);

    while (!g_request_shutdown) {
        ros::spinOnce();
        usleep(500);
    }

    vn200.unregisterAsyncPacketReceivedHandler();
    ros::shutdown();
}

