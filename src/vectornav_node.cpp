/*
 * Copyright 2013 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>


ros::Publisher pub_imu;
ros::Publisher pub_pose;


/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

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
  printf("ASYNC Data:\n"
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

  printf("\n");
 
    
  // Publish IMU Message
  sensor_msgs::Imu msg_imu;
  msg_imu.header.seq = seq;
  msg_imu.header.stamp = timestamp;
  msg_imu.header.frame_id = "imu";

  msg_imu.orientation.x = data->quaternion.x;
  msg_imu.orientation.y = data->quaternion.y;
  msg_imu.orientation.x = data->quaternion.z;
  msg_imu.orientation.w = data->quaternion.w;

  pub_imu.publish(msg_imu);


  // Publish Pose Message
  geometry_msgs::PoseStamped msg_pose;
  msg_pose.header.seq = seq;
  msg_pose.header.stamp = timestamp;
  msg_pose.header.frame_id = "imu";

  msg_pose.pose.orientation = msg_imu.orientation;

  pub_pose.publish(msg_pose);

}


/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS;
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n;
  
  pub_imu  = n.advertise<sensor_msgs::Imu>(          "imu" , 1000);
  pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  
  
  // Initialize VectorNav
	Vn200 vn200;
	vn200_connect(&vn200, COM_PORT, BAUD_RATE);
  vn200_setAsynchronousDataOutputType(&vn200, 19, true); //19, 20, 22
	sleep(1);
	vn200_registerAsyncDataReceivedListener(&vn200, &asyncDataListener);
  
  
  ros::spin();
  

  vn200_unregisterAsyncDataReceivedListener(&vn200, &asyncDataListener);
  vn200_disconnect(&vn200);
  return 0;
}

