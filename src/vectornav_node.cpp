/*
 * Copyright 2013 Dereck Wonnacott <dereck@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include <geometry_msgs/PoseStamped.h> // Should this even be here?


std::string port;
int baud;
ros::Publisher pub_imu;
ros::Publisher pub_magnetic;
ros::Publisher pub_temp;
ros::Publisher pub_pressure;

ros::Publisher pub_gps;
ros::Publisher pub_gps_time;

Vn200 vn200;


void poll_timerCB(const ros::TimerEvent&)
{

    // Only bother if we have subscribers
    if (pub_imu.getNumSubscribers()  <= 0 && pub_magnetic.getNumSubscribers() <= 0 &&
        pub_temp.getNumSubscribers() <= 0 && pub_pressure.getNumSubscribers() <= 0 &&
        pub_gps.getNumSubscribers()  <= 0 && pub_gps_time.getNumSubscribers() <= 0)
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
    msg_gps.header.frame_id = "gps";  // GPS Antenna frame
    
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
        
    msg_gps_time.time_ref.sec  = 315964800 + (604800 * gpsWeek) + gpsTime;
    msg_gps_time.time_ref.nsec = (gpsTime - (long)gpsTime) * 1000000000; 
    
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
      msg_imu.header.frame_id = "imu"; // IMU sensor frame
      
      tf::Quaternion q = tf::createQuaternionFromRPY( M_PI * ypr.c0/180, 
                                                      M_PI * ypr.c1/180, 
                                                      M_PI * ypr.c2/180 );

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
  
  int poll_rate;
  
  // Read Parameters
  n_.param<std::string>("port"     , port     , "/dev/ttyUSB0");
  n_.param<int>(        "baud"     , baud     , 115200);
  n_.param<int>(        "poll_rate", poll_rate, 40);
  
  ROS_DEBUG("%s %d %d Hz", port.c_str(), baud, poll_rate);
  
  // Initialize Publishers  
  pub_imu         = n_.advertise<sensor_msgs::Imu>          ("imu/data"        , 1000);
  pub_magnetic    = n_.advertise<sensor_msgs::MagneticField>("imu/magnetic"    , 1000);
  pub_temp        = n_.advertise<sensor_msgs::Temperature>  ("imu/temperature" , 1000);
  pub_pressure    = n_.advertise<sensor_msgs::FluidPressure>("imu/pressure"    , 1000);
  
  pub_gps         = n_.advertise<sensor_msgs::NavSatFix>    ("gps/fix"         , 1000);
  pub_gps_time    = n_.advertise<sensor_msgs::TimeReference>("gps/time"        , 1000);
  
  
  // Initialize VectorNav
	vn200_connect(&vn200, port.c_str(), baud);
		
	// Polling loop
	ros::Timer poll_timer = n.createTimer(ros::Duration(1.0/(double)poll_rate), poll_timerCB);
	
	// Note: While the VN200 has an Async interface, it can only aquire one 'type' of 
	//       data at a time, which means that you cannot get both the IMU and INS data at 
	//       the same time. The Async method allows a 50Hz update rate. 
	//       It should be reasonbly trivial to rewrite this node to use ASYNC mode, and 
	//       fall back to polling mode when multiple data types are requested.
	
  ros::spin();
	
  vn200_disconnect(&vn200);
  return 0;
}

