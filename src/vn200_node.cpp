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

#include "vectornav.h"

#include <ros/ros.h>
#include <tf/tf.h>

// Message Types
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>


// Parameters
std::string imu_frame_id, gps_frame_id, gps_time_source;
double time_offset;

// Publishers
ros::Publisher pub_imu;
ros::Publisher pub_magnetic;
ros::Publisher pub_temp;
ros::Publisher pub_pressure;

ros::Publisher pub_gps;
ros::Publisher pub_gps_time;

// Device Handle
Vn200 vn200;

void asyncDataListener(Vn200* sender, Vn200CompositeData* data)
{
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
	  "  YPR.Roll:               %+#7.2f\n"

	  "  quaternion.X:           %+#7.2f\n"
	  "  quaternion.Y:           %+#7.2f\n"
	  "  quaternion.Z:           %+#7.2f\n"
	  "  quaternion.W:           %+#7.2f\n"

	  "                          {Value, Voltage}\n"
	  "  magnetic X:             %+#7.2f, %+#7.2f\n"
	  "  magnetic Y:             %+#7.2f, %+#7.2f\n"
	  "  magnetic Z:             %+#7.2f, %+#7.2f\n"

	  "  acceleration X:         %+#7.2f, %+#7.2f\n"
	  "  acceleration Y:         %+#7.2f, %+#7.2f\n"
	  "  acceleration Z:         %+#7.2f, %+#7.2f\n"

	  "                          {Value, Voltage, Bias, BiasVariance}\n"
	  "  angularRate X:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Y:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Z:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"

	  "  Attitude Variance X:    %+#7.2f\n"
	  "  Attitude Variance Y:    %+#7.2f\n"
	  "  Attitude Variance Z:    %+#7.2f\n"

	  "  Direction Cosine Matrix:\n"
	  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "    %+#7.2f, %+#7.2f, %+#7.2f\n"

	  "  Temperature:            %+#7.2f\n"
	  "  Temperature Voltage:    %+#7.2f\n"
	  "  Pressure:               %+#7.2f\n"

	  "  GPS Time:               %+#7.2f\n"
	  "  GPS Week:               %u\n"
	  "  GPS Fix:                %s (%u)\n"
	  "  GPS Satellites:         %u\n"

	  "  LLA.Lattitude:          %+#7.2f\n"
	  "  LLA.Longitude:          %+#7.2f\n"
	  "  LLA.Altitude:           %+#7.2f\n"

	  "  Velocity.North:         %+#7.2f\n"
	  "  Velocity.East:          %+#7.2f\n"
	  "  Velocity.Down:          %+#7.2f\n"

	  "  Position Accuracy X:    %+#7.2f\n"
	  "  Position Accuracy Y:    %+#7.2f\n"
	  "  Position Accuracy Z:    %+#7.2f\n"

	  "  Speed Accuracy:         %+#7.2f\n"
	  "  Time Accuracy:          %+#7.2f\n"

	  "  INS Status:             %7.4X\n"
	  "    %s\n"
	  "    %s\n"
	  "    %s\n"

	  "  Attitude Uncertainty:   %+#7.2f\n"
	  "  Position Uncertainty:   %+#7.2f\n"
	  "  Velocity Uncertainty:   %+#7.2f\n"
	  ,

	  data->ypr.yaw,
    data->ypr.pitch,
    data->ypr.roll,
    
	  data->quaternion.x,
	  data->quaternion.y,
	  data->quaternion.z,
	  data->quaternion.w,

	  data->magnetic.c0, data->magneticVoltage.c0, 
	  data->magnetic.c1, data->magneticVoltage.c1, 
	  data->magnetic.c2, data->magneticVoltage.c2,

	  data->acceleration.c0, data->accelerationVoltage.c0, 
	  data->acceleration.c1, data->accelerationVoltage.c1, 
	  data->acceleration.c2, data->accelerationVoltage.c2,

	  data->angularRate.c0,     data->angularRateVoltage.c0, 
	  data->angularRateBias.c0, data->angularRateBiasVariance.c0, 
	  data->angularRate.c1,     data->angularRateVoltage.c1, 
	  data->angularRateBias.c1, data->angularRateBiasVariance.c1, 
	  data->angularRate.c2,     data->angularRateVoltage.c2,
	  data->angularRateBias.c2, data->angularRateBiasVariance.c2, 

	  data->attitudeVariance.c0, 
	  data->attitudeVariance.c1, 
	  data->attitudeVariance.c2, 

	  data->dcm.c00, data->dcm.c01, data->dcm.c02,
	  data->dcm.c10, data->dcm.c11, data->dcm.c12,
	  data->dcm.c20, data->dcm.c21, data->dcm.c22,

	  data->temperature,
	  data->temperatureVoltage, 
	  data->pressure,    

	  data->gpsTimeOfWeek,
	  data->gpsWeek,
	  gpsFix.c_str(), data->gpsFix,
	  data->numberOfSatellites,

	  data->latitudeLongitudeAltitude.c0,
	  data->latitudeLongitudeAltitude.c1,
	  data->latitudeLongitudeAltitude.c2,

	  data->velocity.c0,
	  data->velocity.c1,
	  data->velocity.c2,

	  data->positionAccuracy.c0,
	  data->positionAccuracy.c1,
	  data->positionAccuracy.c2,

	  data->speedAccuracy,
	  data->timeAccuracy,

	  data->insStatus,
	  INSmode.c_str(),
    INSgpsFix.c_str(),
    INSerror.c_str(),
  
	  data->attitudeUncertainty,
	  data->positionUncertainty,
	  data->velocityUncertainty
	  );

      
    // IMU
    sensor_msgs::Imu msg_imu;
    msg_imu.header.seq = seq;
    msg_imu.header.stamp = timestamp;
    msg_imu.header.frame_id = imu_frame_id; // IMU sensor frame
    
    // Orientation is is unavailable from the device when connected to
    // the ASYNC IMU Sensor packet.
    
    msg_imu.angular_velocity.x = data->angularRate.c0;
    msg_imu.angular_velocity.y = data->angularRate.c1;
    msg_imu.angular_velocity.z = data->angularRate.c2;
    
    msg_imu.linear_acceleration.x = data->acceleration.c0;
    msg_imu.linear_acceleration.y = data->acceleration.c1;
    msg_imu.linear_acceleration.z = data->acceleration.c2;
    
    pub_imu.publish(msg_imu);
    
    
    // Magnetic
    sensor_msgs::MagneticField msg_magnetic;
    msg_magnetic.header = msg_imu.header;
    
    msg_magnetic.magnetic_field.x = data->magnetic.c0;
    msg_magnetic.magnetic_field.y = data->magnetic.c1;
    msg_magnetic.magnetic_field.z = data->magnetic.c2;
    
    pub_magnetic.publish(msg_magnetic);
    
    
    // Temperature
    sensor_msgs::Temperature msg_temp;
    msg_temp.header = msg_imu.header;
    
    msg_temp.temperature = data->temperature;
    
    pub_temp.publish(msg_temp);


    // Pressure
    sensor_msgs::FluidPressure msg_pressure;
    msg_pressure.header = msg_imu.header;
    
    msg_pressure.fluid_pressure = data->pressure / 1000.0; // kPa -> Pascals
    
    pub_pressure.publish(msg_pressure);

}

void poll_timerCB(const ros::TimerEvent&)
{

  ROS_INFO("asdfsad\n");
    // Only bother if we have subscribers
    if (pub_imu.getNumSubscribers()  <= 0 && pub_magnetic.getNumSubscribers() <= 0 &&
        pub_temp.getNumSubscribers() <= 0 && pub_pressure.getNumSubscribers() <= 0 &&
        pub_gps.getNumSubscribers()  <= 0 && pub_gps_time.getNumSubscribers() <= 0)
    {
      return;
    }
    
    static int seq = 0;
    seq++;
    ros::Time timestamp =  ros::Time::now() + ros::Duration(time_offset); 
    
    
    // INS & GPS Shared Data
    double gpsTime;
    unsigned short gpsWeek;
    VnVector3 LLA, nedVelocity;


    // GPS Data (5Hz MAX)
    /*
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
    */
    
    
    // INS Solution Data 
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
    
    
    // GPS Fix
    sensor_msgs::NavSatFix msg_gps;
    msg_gps.header.seq = seq;
    msg_gps.header.stamp = timestamp;
    msg_gps.header.frame_id = gps_frame_id;  // GPS Antenna frame
    
    if (status & 0x4)
      msg_gps.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    else
      msg_gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

    msg_gps.latitude  = LLA.c0;
    msg_gps.longitude = LLA.c1;
    msg_gps.altitude  = LLA.c2;
   
    msg_gps.position_covariance[0] = positionUncertainty;
    msg_gps.position_covariance[1] = 0;
    msg_gps.position_covariance[2] = 0;
    
    msg_gps.position_covariance[3] = 0;
    msg_gps.position_covariance[4] = positionUncertainty;
    msg_gps.position_covariance[5] = 0;
    
    msg_gps.position_covariance[6] = 0;
    msg_gps.position_covariance[7] = 0;
    msg_gps.position_covariance[8] = positionUncertainty;
    
    msg_gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    
    pub_gps.publish(msg_gps);


    // GPS Time
    sensor_msgs::TimeReference msg_gps_time;
    msg_gps_time.header = msg_gps.header;
    
    if ( gps_time_source == "GPS" )   
      msg_gps_time.time_ref.sec  = (604800 * gpsWeek) + gpsTime;
    else
      // TODO: When gpsWeek rolls over, this will fail
      msg_gps_time.time_ref.sec  = 315964800 + (604800 * gpsWeek) + gpsTime;

    msg_gps_time.time_ref.nsec = (gpsTime - (long)gpsTime) * 1000000000; 
    msg_gps_time.source = gps_time_source;
    pub_gps_time.publish(msg_gps_time);
    
    //   
    // IMU Data
    //
    VnVector3 magnetic, acceleration, angularRate;
    float temperature, pressure;
    
    // Only bother if we have subscribers
    if (pub_imu.getNumSubscribers()  || pub_magnetic.getNumSubscribers() ||
        pub_temp.getNumSubscribers() || pub_pressure.getNumSubscribers())
    {
      vn200_getCalibratedSensorMeasurements(  &vn200,
                                              &magnetic,
                                              &acceleration,
                                              &angularRate,
                                              &temperature,
                                              &pressure );
    
      // IMU
      sensor_msgs::Imu msg_imu;
      msg_imu.header.seq = seq;
      msg_imu.header.stamp = timestamp;
      msg_imu.header.frame_id = imu_frame_id; // IMU sensor frame
      
      // TODO: If INS solution is unavailable, fall back to naive algorithm
      
      // ROS uses North-East-Up, Vector Nav uses North-East-Down
      tf::Quaternion q = tf::createQuaternionFromRPY(  M_PI * ypr.c2/180.0, 
                                                      -M_PI * ypr.c1/180.0, 
                                                      -M_PI * ypr.c0/180.0 );

      tf::quaternionTFToMsg(q, msg_imu.orientation);
      
      msg_imu.angular_velocity.x = angularRate.c0;
      msg_imu.angular_velocity.y = angularRate.c1;
      msg_imu.angular_velocity.z = angularRate.c2;
      
      msg_imu.linear_acceleration.x = acceleration.c0;
      msg_imu.linear_acceleration.y = acceleration.c1;
      msg_imu.linear_acceleration.z = acceleration.c2;
      
      pub_imu.publish(msg_imu);
      
      
      // Magnetic
      sensor_msgs::MagneticField msg_magnetic;
      msg_magnetic.header = msg_imu.header;
      
      msg_magnetic.magnetic_field.x = magnetic.c0;
      msg_magnetic.magnetic_field.y = magnetic.c1;
      msg_magnetic.magnetic_field.z = magnetic.c2;
      
      pub_magnetic.publish(msg_magnetic);
      
      
      // Temperature
      sensor_msgs::Temperature msg_temp;
      msg_temp.header = msg_imu.header;
      
      msg_temp.temperature = temperature;
      
      pub_temp.publish(msg_temp);


      // Pressure
      sensor_msgs::FluidPressure msg_pressure;
      msg_pressure.header = msg_imu.header;
      
      msg_pressure.fluid_pressure = pressure / 1000.0; // kPa -> Pascals
      
      pub_pressure.publish(msg_pressure);
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
  
  n_.param<std::string>("imu/frame_id", imu_frame_id, "imu");
  n_.param<std::string>("gps/frame_id", gps_frame_id, "gps");
  
  n_.param<std::string>("time_reference_source", gps_time_source, "UTC");
  n_.param<double>(     "time_offset"          , time_offset    , 0.0);
   
  // Type: 0 None, 19 IMU, 20 GPS, 22 INS
  n_.param<int>(        "async_output_type"  , async_output_type, 0);
  n_.param<int>(        "async_output_rate"  , async_output_rate, 50); 
  
  // Initialize Publishers
  pub_imu         = n_.advertise<sensor_msgs::Imu>          ("imu/data"        , 1000);
  pub_magnetic    = n_.advertise<sensor_msgs::MagneticField>("imu/magnetic"    , 1000);
  pub_temp        = n_.advertise<sensor_msgs::Temperature>  ("imu/temperature" , 1000);
  pub_pressure    = n_.advertise<sensor_msgs::FluidPressure>("imu/pressure"    , 1000);
  
  pub_gps         = n_.advertise<sensor_msgs::NavSatFix>    ("gps/fix"         , 1000);
  pub_gps_time    = n_.advertise<sensor_msgs::TimeReference>("gps/time"        , 1000);
  
  
  // Initialize VectorNav
  ROS_INFO("Initializing vn200. Port:%s Baud:%d\n", port.c_str(), baud);
	vn200_connect(&vn200, port.c_str(), baud); 
  vn200_setAsynchronousDataOutputType(&vn200, async_output_type, true);
		
	if (async_output_type == 0)
	{
	  // Polling loop
    ROS_INFO("Polling at %d Hz\n", poll_rate);
	  ros::Timer poll_timer = n.createTimer(ros::Duration(1.0/(double)poll_rate), poll_timerCB);
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

