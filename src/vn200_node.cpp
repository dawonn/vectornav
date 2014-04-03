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

// Message Types
#include <vectornav/gps.h>
#include <vectornav/ins.h>
#include <vectornav/sensors.h>

// Params
std::string imu_frame_id, gps_frame_id;

// Publishers
ros::Publisher pub_ins;
ros::Publisher pub_gps;
ros::Publisher pub_sensors;

// Device
Vn200 vn200;

void asyncDataListener(Vn200* sender, Vn200CompositeData* data)
{
  //TODO: Publish messages perhaps? ;)

  ros::Time timestamp =  ros::Time::now();
  static int seq = 0;
  seq++;
  
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
  
  // INS Status
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


void poll_device()
{
    // Only bother if we have subscribers
    if (pub_ins.getNumSubscribers()     <= 0 &&
        pub_gps.getNumSubscribers()     <= 0 &&
        pub_sensors.getNumSubscribers() <= 0)
    {
      return;
    }
    
    static int seq = 0;
    seq++;
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
      msg_gps.header.seq      = seq;
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
      msg_ins.header.seq      = seq;
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
      
      
      msg_ins.AttUncerainty = attitudeUncertainty;
      msg_ins.PosUncerainty  = positionUncertainty;
      msg_ins.VelUncerainty  = velocityUncertainty;
      
      pub_ins.publish(msg_ins);
    }
    
    
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
      msg_sensors.header.seq      = seq;
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

void poll_timerCB(const ros::TimerEvent&)
{
  poll_device();
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


/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS;
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n; 
  ros::NodeHandle n_("~");
  
  // Read Parameters
  std::string port;
  int baud, poll_rate, async_output_type, async_output_rate;
  
  n_.param<std::string>("serial_port" , port     , "/dev/ttyUSB0");
  n_.param<int>(        "serial_baud" , baud     , 115200);
  n_.param<int>(        "poll_rate"   , poll_rate, 40);
  
  n_.param<std::string>("imu/frame_id", imu_frame_id, "LLA");
  n_.param<std::string>("gps/frame_id", gps_frame_id, "LLA");
   
  // Type: 0 None, 19 IMU, 20 GPS, 22 INS
  n_.param<int>(        "async_output_type"  , async_output_type, 0);
  n_.param<int>(        "async_output_rate"  , async_output_rate, 50); 
  
  // Initialize Publishers
  pub_ins     = n_.advertise<vectornav::ins>    ("ins", 1000);
  pub_gps     = n_.advertise<vectornav::gps>    ("gps", 1000);
  pub_sensors = n_.advertise<vectornav::sensors>("imu", 1000);
  
  // Initialize VectorNav
  VN_ERROR_CODE vn_retval;
  char vn_error_msg[100];
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
	
  vn_retval = vn200_setAsynchronousDataOutputType(&vn200, async_output_type, true);
	if (vn_retval != VNERR_NO_ERROR)
  {
    vnerr_msg(vn_retval, vn_error_msg);
	  ROS_FATAL( "Could not set output type on device via: %s, Error Text: %s", port.c_str(), vn_error_msg);
	  exit (EXIT_FAILURE);
	}
	
  ros::Timer poll_timer; 
	if (async_output_type == 0)
	{
	  // Polling loop
    ROS_INFO("Polling at %d Hz\n", poll_rate);
	  poll_timer = n.createTimer(ros::Duration(1.0/(double)poll_rate), poll_timerCB);
  }
  else
  {
    // Async Request
    switch(async_output_rate)
    {
      case 1:
      case 2:
      case 4:
      case 5:
      case 10:
      case 20:
      case 25:
      case 40:
      case 50:
      case 100:
      case 200:
        ROS_INFO("Establishing an ASYNC subscription at %d Hz\n", async_output_rate);
        break;
      default:
        ROS_ERROR("Invalid ASYNC rate specified (%d). "
                  "Valid rates: {1,2,4,5,10,25,40,50,100,200}", async_output_rate);
    }
    vn200_setAsynchronousDataOutputFrequency(&vn200, async_output_rate, true);
	  vn200_registerAsyncDataReceivedListener(&vn200, &asyncDataListener);
	}
	
  ros::spin();
	
  vn200_disconnect(&vn200);
  return 0;
}

