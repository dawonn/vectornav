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

#include "vectornav.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>


/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;


/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS;
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n;
  
  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu", 1000);
  
  
  // Initialize VectorNav
	double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLognitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;

	Vn200 vn200;
	int i;

	vn200_connect(&vn200, COM_PORT, BAUD_RATE);
  
  
  // Polling loop
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // Get IMU Data
		vn200_getInsSolution(
			&vn200,
			&gpsTime,
			&gpsWeek,
			&status,
			&ypr,
			&latitudeLognitudeAltitude,
			&nedVelocity,
			&attitudeUncertainty,
			&positionUncertainty,
			&velocityUncertainty);


    // Debug message
		printf("INS Solution:\n"
			"  GPS Time:               %f\n"
			"  GPS Week:               %u\n"
			"  INS Status:             %.4X\n"
			"  YPR.Yaw:                %+#7.2f\n"
			"  YPR.Pitch:              %+#7.2f\n"
			"  YPR.Roll:               %+#7.2f\n"
			"  LLA.Lattitude:          %+#7.2f\n"
			"  LLA.Longitude:          %+#7.2f\n"
			"  LLA.Altitude:           %+#7.2f\n"
			"  Velocity.North:         %+#7.2f\n"
			"  Velocity.East:          %+#7.2f\n"
			"  Velocity.Down:          %+#7.2f\n"
			"  Attitude Uncertainty:   %+#7.2f\n"
			"  Position Uncertainty:   %+#7.2f\n"
			"  Velocity Uncertainty:   %+#7.2f\n",
			gpsTime,
			gpsWeek,
			status,
			ypr.c0,
			ypr.c1,
			ypr.c2,
			latitudeLognitudeAltitude.c0,
			latitudeLognitudeAltitude.c1,
			latitudeLognitudeAltitude.c2,
			nedVelocity.c0,
			nedVelocity.c1,
			nedVelocity.c2,
			attitudeUncertainty,
			positionUncertainty,
			velocityUncertainty);

		printf("\n\n");
  
  
    // Publish IMU message
    sensor_msgs::Imu msg_imu;
    
    tf::Quaternion q = tf::createQuaternionFromRPY( ypr.c0, ypr.c1, ypr.c2 );
    tf::quaternionTFToMsg(q, msg_imu.orientation);
    
    pub_imu.publish(msg_imu);
  
  
    // Rinse and repeat. :)
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  vn200_disconnect(&vn200);

	return 0;
}

