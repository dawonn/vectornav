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

#include "vectornav.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <signal.h>
// Message Types
#include <vectornav/utc_time.h>
#include <vectornav/gps.h>
#include <vectornav/ins.h>
#include <vectornav/sensors.h>

#include <vectornav/sync_in.h>
#include <ros/xmlrpc_manager.h>

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
Vn200 vn200;

int ins_seq           = 0;
int imu_seq           = 0;
int gps_seq           = 0;
int sync_in_seq       = 0;
int msg_cnt           = 0;
int last_group_number = 0;


std::string port;
char vn_error_msg[100];

struct ins_binary_data_struct 
{
    uint64_t gps_time;
    uint64_t sync_in_time;
    float yaw;
    float pitch;
    float roll;
    double latitude;
    double longitude;
    double altitude;
    float vel_north;
    float vel_east;
    float vel_down;
    uint16_t ins_status;
    uint32_t sync_in_count;
    float yaw_sigma;
    float pitch_sigma;
    float roll_sigma;
    float pos_sigma;
    float vel_sigma;
} __attribute__((packed));

ins_binary_data_struct ins_binary_data;

struct utc_time_struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} __attribute__((packed));

struct gps_binary_data_struct
{
    utc_time_struct utc_time;
    uint64_t tow;
    uint16_t week;
    uint8_t  num_sats;
    uint8_t  fix;
    double latitude;
    double longitude;
    double altitude;
    float vel_north;
    float vel_east;
    float vel_down;
    float north_sigma;
    float east_sigma;
    float down_sigma;
    float vel_sigma;
    uint32_t time_sigma;
} __attribute__((packed));

struct gps_binary_data_struct gps_binary_data;

struct imu_binary_data_struct
{
    uint64_t gps_time;
    float accel_x;
    float accel_y;
    float accel_z;
    float rotr_x;
    float rotr_y;
    float rotr_z;
} __attribute__((packed));

struct imu_binary_data_struct imu_binary_data;

int ins_msg = 0;
int gps_msg = 0;
int imu_msg = 0;

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

        msg_gps.LLA.x = gps_binary_data.latitude;
        msg_gps.LLA.y = gps_binary_data.longitude;
        msg_gps.LLA.z = gps_binary_data.altitude;

        msg_gps.NedVel.x = gps_binary_data.vel_north;
        msg_gps.NedVel.y = gps_binary_data.vel_east;
        msg_gps.NedVel.z = gps_binary_data.vel_down;

        msg_gps.NedAcc.x = gps_binary_data.north_sigma;
        msg_gps.NedAcc.y = gps_binary_data.east_sigma;
        msg_gps.NedAcc.z = gps_binary_data.down_sigma;

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

        msg_ins.RPY.x = ins_binary_data.roll; // Intentional re-ordering
        msg_ins.RPY.y = ins_binary_data.pitch;
        msg_ins.RPY.z = ins_binary_data.yaw;

        msg_ins.LLA.x = ins_binary_data.latitude;
        msg_ins.LLA.y = ins_binary_data.longitude;
        msg_ins.LLA.z = ins_binary_data.altitude;

        msg_ins.NedVel.x = ins_binary_data.vel_north;
        msg_ins.NedVel.y = ins_binary_data.vel_east;
        msg_ins.NedVel.z = ins_binary_data.vel_down;


        msg_ins.RollUncertainty = ins_binary_data.roll_sigma;
        msg_ins.PitchUncertainty = ins_binary_data.pitch_sigma;
        msg_ins.YawUncertainty = ins_binary_data.yaw_sigma;
        msg_ins.PosUncertainty  = ins_binary_data.pos_sigma;
        msg_ins.VelUncertainty  = ins_binary_data.vel_sigma;

        pub_ins.publish(msg_ins);
    } 
}

void publish_imu_data()
{
    imu_seq++;
    ros::Time timestamp =  ros::Time::now(); 
    // IMU Data
    if (pub_sensors.getNumSubscribers() > 0)
    {
        vectornav::sensors msg_sensors;
        msg_sensors.header.seq      = imu_seq;
        msg_sensors.header.stamp    = timestamp;
        msg_sensors.header.frame_id = "imu";
        msg_sensors.gps_time 	    = (double)imu_binary_data.gps_time*1E-9;
        msg_sensors.Accel.x = imu_binary_data.accel_x;
        msg_sensors.Accel.y = imu_binary_data.accel_y;
        msg_sensors.Accel.z = imu_binary_data.accel_z;

        msg_sensors.Gyro.x = imu_binary_data.rotr_x;
        msg_sensors.Gyro.y = imu_binary_data.rotr_y;
        msg_sensors.Gyro.z = imu_binary_data.rotr_z;

        pub_sensors.publish(msg_sensors);
    }
}

void publish_sync_in()
{
    sync_in_seq++;
    ros::Time timestamp =  ros::Time::now(); 
    // sync_in from camera strobe
    if (pub_sync_in.getNumSubscribers() > 0)
    {
        vectornav::sync_in msg_sync_in;
        msg_sync_in.header.seq      = imu_seq;
        msg_sync_in.header.stamp    = timestamp;
        msg_sync_in.header.frame_id = "sync_in";
        msg_sync_in.gps_time 	    = (double)(ins_binary_data.gps_time-ins_binary_data.sync_in_time)*1E-9;
        msg_sync_in.sync_in_count   = ins_binary_data.sync_in_count;

        pub_sync_in.publish(msg_sync_in);
    }
}

void asyncBinaryResponseListener(Vn200* sender, unsigned char* data, unsigned int buf_len)
{
    int sync_byte = data[0];
    int group_number = data[1];
    int fields = data[3]*256+data[2];
    static uint32_t last_sync_in_count = 0;

    //ROS_INFO_STREAM(sync_byte << " " << group_number);

    if (sync_byte == 250 && group_number == 1)
    {
        imu_msg++;
        memcpy(&imu_binary_data, data+4, buf_len-6);
        publish_imu_data();
        if (remainder(imu_msg, 500) == 0)
        {
            imu_msg = 0;
            ROS_INFO_STREAM("IMU_Time: " << imu_binary_data.gps_time*1E-9 << " rotr_x: " <<
                    imu_binary_data.rotr_x << " rotr_y: " << imu_binary_data.rotr_y <<
                    " rotr_z: " << imu_binary_data.rotr_z << " accel_x: " <<
                    imu_binary_data.accel_x << " accel_y: " << imu_binary_data.accel_y <<
                    " accel_z: " << imu_binary_data.accel_z);
        }
    }
    else if (sync_byte == 250 && group_number == 49)
    {
        ins_msg++;
        memcpy(&ins_binary_data, data+8, buf_len-10);
        publish_ins_data();

        if (last_sync_in_count != ins_binary_data.sync_in_count)
        {
            double syncInTime = (ins_binary_data.gps_time - ins_binary_data.sync_in_time) * 1e-9;
            ROS_DEBUG_STREAM("Received strobe count:" << ins_binary_data.sync_in_count << " at GPS time " << std::fixed << std::setw(12) << syncInTime);
            last_sync_in_count = ins_binary_data.sync_in_count;
            publish_sync_in();
        }

        if (remainder(ins_msg, 100) == 0)
        {
            ins_msg = 0;
            ROS_INFO_STREAM("INS_Time: " << ins_binary_data.gps_time*1E-9 << " Yaw: " <<
                    ins_binary_data.yaw << " Pitch: " << ins_binary_data.pitch <<
                    " Roll: " << ins_binary_data.roll << " INS_Status: " <<
                    ins_binary_data.ins_status << " latitude: " << ins_binary_data.latitude <<
                    " longitude: " << ins_binary_data.longitude << " altitude: " << ins_binary_data.altitude);
        }
    }
    else if (sync_byte == 250 && group_number == 8)
    {
        gps_msg++;
        memcpy(&gps_binary_data, data+4, buf_len-6);

        publish_gps_data();
        if (remainder(gps_msg, 16) == 0)
        {
            gps_msg = 0;
            ROS_INFO_STREAM("TOW: " << gps_binary_data.tow*1E-9 << " NumSats: " << (int)gps_binary_data.num_sats);
        }

    }

    /*    if (group_number != last_group_number)

          ROS_INFO_STREAM("sync byte: " << sync_byte << "group number: " << group_number << " fields: " << fields);
          last_group_number = group_number;
          } */
}

void asyncDataListener(Vn200* sender, Vn200CompositeData* data)
{
    //TODO: Publish messages perhaps? ;)



    // GPS Fix string value
    std::string gpsFix;
    switch(data->gpsFix)
    {
        case 0:
            gpsFix = "No Fix";
            break;
        case 1:
            gpsFix = "Time Only";
            break;
        case 2:
            gpsFix = "2D Fix";
            break;
        case 3:
            gpsFix = "3D Fix";
            break;
        default:
            gpsFix = "Unknown";
    }

    std::string INSmode;
    switch (data->insStatus & 0x3)
    {
        case 0:
            INSmode = "Not Tracking";
            break;
        case 1:
            INSmode = "Degraded Performance";
            break;
        case 2:
            INSmode = "Perfomance with Spec";
            break;
        default:
            INSmode = "Unknown";
    }

    std::string INSgpsFix;
    if(data->insStatus & 0x8)
        INSgpsFix = "GPS Fixed";
    else
        INSgpsFix = "GPS Not Fixed";

    std::string INSerror;
    if(data->insStatus & 0x78)
        INSerror = "INS Internal Error";
    else
        INSerror = "INS Systems No Error";


    // Debug message
    ROS_INFO("\nASYNC Data:\n"
            "  YPR.Yaw:                %+#7.2f\n"
            "  YPR.Pitch:              %+#7.2f\n"
            "  YPR.Roll:               %+#7.2f\n" ,
            data->ypr.yaw,
            data->ypr.pitch,
            data->ypr.roll);

    ROS_INFO(
            "\n  quaternion.X:           %+#7.2f\n"
            "  quaternion.Y:           %+#7.2f\n"
            "  quaternion.Z:           %+#7.2f\n"
            "  quaternion.W:           %+#7.2f\n",
            data->quaternion.x,
            data->quaternion.y,
            data->quaternion.z,
            data->quaternion.w);

    ROS_INFO(
            "\n                          {Value, Voltage}\n"
            "  magnetic X:             %+#7.2f, %+#7.2f\n"
            "  magnetic Y:             %+#7.2f, %+#7.2f\n"
            "  magnetic Z:             %+#7.2f, %+#7.2f\n",
            data->magnetic.c0, data->magneticVoltage.c0, 
            data->magnetic.c1, data->magneticVoltage.c1, 
            data->magnetic.c2, data->magneticVoltage.c2);

    ROS_INFO(
            "\n  acceleration X:         %+#7.2f, %+#7.2f\n"
            "  acceleration Y:         %+#7.2f, %+#7.2f\n"
            "  acceleration Z:         %+#7.2f, %+#7.2f\n",
            data->acceleration.c0, data->accelerationVoltage.c0, 
            data->acceleration.c1, data->accelerationVoltage.c1, 
            data->acceleration.c2, data->accelerationVoltage.c2);

    ROS_INFO(
            "\n                          {Value, Voltage, Bias, BiasVariance}\n"
            "  angularRate X:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
            "  angularRate Y:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
            "  angularRate Z:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n",
            data->angularRate.c0,     data->angularRateVoltage.c0, 
            data->angularRateBias.c0, data->angularRateBiasVariance.c0, 
            data->angularRate.c1,     data->angularRateVoltage.c1, 
            data->angularRateBias.c1, data->angularRateBiasVariance.c1, 
            data->angularRate.c2,     data->angularRateVoltage.c2,
            data->angularRateBias.c2, data->angularRateBiasVariance.c2);

    ROS_INFO(
            "\n  Attitude Variance X:    %+#7.2f\n"
            "  Attitude Variance Y:    %+#7.2f\n"
            "  Attitude Variance Z:    %+#7.2f\n",
            data->attitudeVariance.c0, 
            data->attitudeVariance.c1, 
            data->attitudeVariance.c2);

    ROS_INFO(
            "\n  Direction Cosine Matrix:\n"
            "    %+#7.2f, %+#7.2f, %+#7.2f\n"
            "    %+#7.2f, %+#7.2f, %+#7.2f\n"
            "    %+#7.2f, %+#7.2f, %+#7.2f\n",
            data->dcm.c00, data->dcm.c01, data->dcm.c02,
            data->dcm.c10, data->dcm.c11, data->dcm.c12,
            data->dcm.c20, data->dcm.c21, data->dcm.c22);

    ROS_INFO(
            "\n  Temperature:            %+#7.2f\n"
            "  Temperature Voltage:    %+#7.2f\n"
            "  Pressure:               %+#7.2f\n",
            data->temperature,
            data->temperatureVoltage, 
            data->pressure);

    ROS_INFO(
            "\n  GPS Time:               %+#7.2f\n"
            "  GPS Week:               %u\n"
            "  GPS Fix:                %s (%u)\n"
            "  GPS Satellites:         %u\n",
            data->gpsTimeOfWeek,
            data->gpsWeek,
            gpsFix.c_str(), data->gpsFix,
            data->numberOfSatellites);

    ROS_INFO(
            "\n  LLA.Lattitude:          %+#7.2f\n"
            "  LLA.Longitude:          %+#7.2f\n"
            "  LLA.Altitude:           %+#7.2f\n",
            data->latitudeLongitudeAltitude.c0,
            data->latitudeLongitudeAltitude.c1,
            data->latitudeLongitudeAltitude.c2);

    ROS_INFO(
            "\n  Velocity.North:         %+#7.2f\n"
            "  Velocity.East:          %+#7.2f\n"
            "  Velocity.Down:          %+#7.2f\n",
            data->velocity.c0,
            data->velocity.c1,
            data->velocity.c2);

    ROS_INFO(
            "\n  Position Accuracy X:    %+#7.2f\n"
            "  Position Accuracy Y:    %+#7.2f\n"
            "  Position Accuracy Z:    %+#7.2f\n",
            data->positionAccuracy.c0,
            data->positionAccuracy.c1,
            data->positionAccuracy.c2);

    ROS_INFO(
            "\n  Speed Accuracy:         %+#7.2f\n"
            "  Time Accuracy:          %+#7.2f\n",
            data->speedAccuracy,
            data->timeAccuracy);

    ROS_INFO(
            "\n  INS Status:             %7.4X\n"
            "    %s\n"
            "    %s\n"
            "    %s\n",
            data->insStatus,
            INSmode.c_str(),
            INSgpsFix.c_str(),
            INSerror.c_str());

    ROS_INFO(
            "\n  Attitude Uncertainty:   %+#7.2f\n"
            "  Position Uncertainty:   %+#7.2f\n"
            "  Velocity Uncertainty:   %+#7.2f\n",
            data->attitudeUncertainty,
            data->positionUncertainty,
            data->velocityUncertainty);

}

void poll_gps()
{
    // Only bother if we have subscribers
    if ( pub_gps.getNumSubscribers() <= 0 )
    {
        return;
    }

    gps_seq++;
    ros::Time timestamp =  ros::Time::now(); 

    // INS & GPS Shared Data
    double gpsTime;
    unsigned short gpsWeek;
    VnVector3 LLA, nedVelocity;


    // GPS Data (5Hz MAX)
    if (pub_gps.getNumSubscribers() > 0)
    {
        unsigned char gpsFix, numberOfSatellites;
        float speedAccuracy, timeAccuracy;
        VnVector3 positionAccuracy;

        vn200_getGpsSolution( &vn200,
                &gpsTime,
                &gpsWeek,
                &gpsFix,
                &numberOfSatellites,
                &LLA,
                &nedVelocity,
                &positionAccuracy,
                &speedAccuracy,
                &timeAccuracy ); 

        vectornav::gps msg_gps;
        msg_gps.header.seq      = gps_seq;
        msg_gps.header.stamp    = timestamp;
        msg_gps.header.frame_id = "gps";

        msg_gps.Time    = gpsTime;
        msg_gps.Week    = gpsWeek;
        msg_gps.GpsFix  = gpsFix;
        msg_gps.NumSats = numberOfSatellites;

        msg_gps.LLA.x = LLA.c0;
        msg_gps.LLA.y = LLA.c1;
        msg_gps.LLA.z = LLA.c2;

        msg_gps.NedVel.x = nedVelocity.c0;
        msg_gps.NedVel.y = nedVelocity.c1;
        msg_gps.NedVel.z = nedVelocity.c2;

        msg_gps.NedAcc.x = positionAccuracy.c0;
        msg_gps.NedAcc.y = positionAccuracy.c1;
        msg_gps.NedAcc.z = positionAccuracy.c2;

        msg_gps.SpeedAcc = speedAccuracy;
        msg_gps.TimeAcc  = timeAccuracy;

        pub_gps.publish(msg_gps);
    }
}

void poll_ins()
{
    // Only bother if we have subscribers
    if ( pub_ins.getNumSubscribers() <= 0 )
    {
        return;
    }

    ins_seq++;
    ros::Time timestamp =  ros::Time::now(); 

    // INS & GPS Shared Data
    double gpsTime;
    unsigned short gpsWeek;
    VnVector3 LLA, nedVelocity;

    // INS Solution Data 
    if (pub_ins.getNumSubscribers() > 0)
    {
        unsigned short  status;
        VnVector3 ypr;
        float attitudeUncertainty, positionUncertainty, velocityUncertainty;

        vn200_getInsSolution( &vn200,
                &gpsTime,
                &gpsWeek,
                &status,
                &ypr,
                &LLA,
                &nedVelocity,
                &attitudeUncertainty,
                &positionUncertainty,
                &velocityUncertainty );            

        vectornav::ins msg_ins;
        msg_ins.header.seq      = ins_seq;
        msg_ins.header.stamp    = timestamp;
        msg_ins.header.frame_id = imu_frame_id;

        msg_ins.Time    = gpsTime;
        msg_ins.Week    = gpsWeek;
        msg_ins.Status  = status;

        msg_ins.RPY.x = ypr.c2; // Intentional re-ordering
        msg_ins.RPY.y = ypr.c1;
        msg_ins.RPY.z = ypr.c0;

        msg_ins.LLA.x = LLA.c0;
        msg_ins.LLA.y = LLA.c1;
        msg_ins.LLA.z = LLA.c2;

        msg_ins.NedVel.x = nedVelocity.c0;
        msg_ins.NedVel.y = nedVelocity.c1;
        msg_ins.NedVel.z = nedVelocity.c2;


        msg_ins.YawUncertainty = attitudeUncertainty;
        msg_ins.PosUncertainty  = positionUncertainty;
        msg_ins.VelUncertainty  = velocityUncertainty;

        pub_ins.publish(msg_ins);
    }
}

void poll_imu()
{
    // Only bother if we have subscribers
    if ( pub_sensors.getNumSubscribers() <= 0)
    {
        return;
    }

    imu_seq++;
    ros::Time timestamp =  ros::Time::now(); 

    // IMU Data
    if (pub_sensors.getNumSubscribers() > 0)
    {
        VnVector3 magnetic, acceleration, angularRate;
        float temperature, pressure;

        vn200_getCalibratedSensorMeasurements(  &vn200,
                &magnetic,
                &acceleration,
                &angularRate,
                &temperature,
                &pressure );

        vectornav::sensors msg_sensors;
        msg_sensors.header.seq      = imu_seq;
        msg_sensors.header.stamp    = timestamp;
        msg_sensors.header.frame_id = imu_frame_id;

        msg_sensors.Mag.x = magnetic.c0;
        msg_sensors.Mag.y = magnetic.c1;
        msg_sensors.Mag.z = magnetic.c2;

        msg_sensors.Accel.x = acceleration.c0;
        msg_sensors.Accel.y = acceleration.c1;
        msg_sensors.Accel.z = acceleration.c2;

        msg_sensors.Gyro.x = angularRate.c0;
        msg_sensors.Gyro.y = angularRate.c1;
        msg_sensors.Gyro.z = angularRate.c2;

        msg_sensors.Temp     = temperature;
        msg_sensors.Pressure = pressure;

        pub_sensors.publish(msg_sensors);

    }
}

void poll_timer_gps_CB(const ros::TimerEvent&)
{
    poll_gps();
}

void poll_timer_ins_CB(const ros::TimerEvent&)
{
    poll_ins();
}

void poll_timer_imu_CB(const ros::TimerEvent&)
{
    poll_imu();
}

void vnerr_msg(VN_ERROR_CODE vn_error, char* msg)
{
    switch(vn_error)
    {
        case VNERR_NO_ERROR:
            strcpy(msg, "No Error");
            break;
        case VNERR_UNKNOWN_ERROR:
            strcpy(msg, "Unknown Error");
            break;
        case VNERR_NOT_IMPLEMENTED:
            strcpy(msg, "Not implemented");
            break;
        case VNERR_TIMEOUT:
            strcpy(msg, "Timemout");
            break;
        case VNERR_INVALID_VALUE:
            strcpy(msg, "Invalid value");
            break;
        case VNERR_FILE_NOT_FOUND:
            strcpy(msg, "File not found");
            break;
        case VNERR_NOT_CONNECTED:
            strcpy(msg, "Not connected");
            break;
        default:
            strcpy(msg, "Undefined Error");
    }
}

void stop_vn200()
{
    VN_ERROR_CODE vn_retval;
    int retry_cnt = 0;

    while (vn_retval != VNERR_NO_ERROR && retry_cnt < 3)
    {
        retry_cnt++;    
        vn_retval = vn200_setBinaryOutputRegisters(&vn200, 0, 1,
            1, 1, true);
    }

    if (vn_retval != VNERR_NO_ERROR)
    {
        vnerr_msg(vn_retval, vn_error_msg);
        ROS_FATAL( "Could not turn off BinaryResponseListener output on device via: %s, Error Text: %s", port.c_str(), vn_error_msg);
        exit (EXIT_FAILURE);
    }

    ROS_INFO("Disabled binary output register");
    vn200_unregisterAsyncBinaryResponseListener(&vn200, &asyncBinaryResponseListener);   
    vn200_disconnect(&vn200);
}

void mySigintHandler(int sig)
{
    g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;

    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
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

    // Initialize Publishers
    pub_ins     = n_.advertise<vectornav::ins>    ("ins", 1000);
    pub_gps     = n_.advertise<vectornav::gps>    ("gps", 1000);
    pub_sensors = n_.advertise<vectornav::sensors>("imu", 1000);
    pub_sync_in = n_.advertise<vectornav::sync_in>("sync_in", 1000);


    // Initialize VectorNav
    VN_ERROR_CODE vn_retval;
    ROS_INFO("Initializing vn200. Port:%s Baud:%d\n", port.c_str(), baud);

    vn_retval = vn200_connect(&vn200, port.c_str(), baud);  
    if (vn_retval != VNERR_NO_ERROR)
    {
        vnerr_msg(vn_retval, vn_error_msg);
        ROS_FATAL("Could not conenct to vn200 on port:%s @ Baud:%d; Error %d \n"
                "Did you add your user to the 'dialout' group in /etc/group?", 
                port.c_str(), 
                baud, 
                vn_retval);
        exit (EXIT_FAILURE);
    }

    usleep(10000);

    vn200_registerAsyncBinaryResponseListener(&vn200, &asyncBinaryResponseListener);   
 
    /* turn off asynchronous ASCII output, retry a couple of times */
    vn_retval = vn200_setAsynchronousDataOutputType(&vn200, 0, true);   

    while (vn_retval != VNERR_NO_ERROR && retry_cnt < 3)
    {
        retry_cnt++;    
    	vn_retval = vn200_setAsynchronousDataOutputType(&vn200, 0, true);   
    }

    if (vn_retval != VNERR_NO_ERROR)
    {
        vnerr_msg(vn_retval, vn_error_msg);
        ROS_FATAL( "Could not set output type on device via: %s, Error Text: %s", port.c_str(), vn_error_msg);
        exit (EXIT_FAILURE);
    }

    ROS_INFO("About to set SynchronizationControl");

    vn_retval = vn200_setSynchronizationControl(&vn200, 
         3, // SyncInMode: 5=Output asynchronous message on trigger of SYNC_IN
         0, // SyncInEdge: 0=rising
         0, // SyncInSkipFactor
         0, // reserved
         6, // SyncOutMode: 6= Trigger on a GPS PPS event (1 Hz) when a 3D fix is valid
         1, // SyncOutPolarity: 0=Negative 1=Positive 
         0, // SyncOutSkipFactor
         100000000, // pulse width in ns
         0, true); // reserved, wait for response

    if (vn_retval != VNERR_NO_ERROR)
    {
        vnerr_msg(vn_retval, vn_error_msg);
        ROS_FATAL( "Could not set SynchronizationControl, Error Text: %s", vn_error_msg);
        exit (EXIT_FAILURE);
    } else {
        ROS_INFO("Set SynchronizationControl");
    }
    
    VnVector3 position;
    position.c0 = 0;
    position.c1 = -0.039;
    position.c2 = 0;

    usleep(10000);

    vn_retval = vn200_setGpsAntennaOffset(&vn200, position, true);


    if (vn_retval != VNERR_NO_ERROR)
    {
        vnerr_msg(vn_retval, vn_error_msg);
        ROS_FATAL( "Could not set GPS Antenna Offset, Error Text: %s", vn_error_msg);
        exit (EXIT_FAILURE);
    }

    usleep(10000);


    vn_retval = vn200_setBinaryOutputRegisters(&vn200, binary_data_output_port, binary_gps_data_rate,
            binary_ins_data_rate, binary_imu_data_rate, true);

    if (vn_retval != VNERR_NO_ERROR)
    {
        vnerr_msg(vn_retval, vn_error_msg);
        ROS_FATAL( "Could not set BinaryResponseListener output on device via: %s, Error Text: %s", port.c_str(), vn_error_msg);
        exit (EXIT_FAILURE);
    }

    while (!g_request_shutdown)
    {
        ros::spinOnce();
        usleep(500);
    }
    stop_vn200();
    ros::shutdown();
    return 0;
}


