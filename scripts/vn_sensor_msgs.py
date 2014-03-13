#!/usr/bin/env python
# 
# Publish ROS standard messages for Vectornav sensors
#
# Copyright (c) 2013 Dereck Wonnacott <dereck@gmail.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# 

from copy             import deepcopy

import rospy
import tf

from sensor_msgs.msg  import Imu
from sensor_msgs.msg  import MagneticField
from sensor_msgs.msg  import Temperature
from sensor_msgs.msg  import FluidPressure

from sensor_msgs.msg  import NavSatFix
from sensor_msgs.msg  import NavSatStatus
from sensor_msgs.msg  import TimeReference

from vectornav.msg    import ins
from vectornav.msg    import sensors

import math



def sub_imuCB(msg_in): 
  global pub_imu
  global pub_mag
  global pub_temp
  global pub_baro
  
  global msg_imu
  msg_imu.header.stamp          = msg_in.header.stamp
  msg_imu.header.frame_id       = msg_in.header.frame_id
  msg_imu.angular_velocity.x    = msg_in.Gyro.x
  msg_imu.angular_velocity.y    = msg_in.Gyro.y
  msg_imu.angular_velocity.z    = msg_in.Gyro.z
  msg_imu.linear_acceleration.x = msg_in.Accel.x
  msg_imu.linear_acceleration.y = msg_in.Accel.y
  msg_imu.linear_acceleration.z = msg_in.Accel.z
  pub_imu.publish(msg_imu)               
  
  msg_mag = MagneticField()
  msg_mag.header.stamp     = msg_in.header.stamp
  msg_mag.header.frame_id  = msg_in.header.frame_id
  msg_mag.magnetic_field.x = msg_in.Mag.x
  msg_mag.magnetic_field.y = msg_in.Mag.y
  msg_mag.magnetic_field.z = msg_in.Mag.z
  pub_mag.publish(msg_mag)
  
  msg_temp = Temperature()
  msg_temp.header.stamp     = msg_in.header.stamp
  msg_temp.header.frame_id  = msg_in.header.frame_id
  msg_temp.temperature      = msg_in.Temp
  pub_temp.publish(msg_temp)
  
  msg_baro = FluidPressure()
  msg_baro.header.stamp     = msg_in.header.stamp
  msg_baro.header.frame_id  = msg_in.header.frame_id
  msg_baro.fluid_pressure   = msg_in.Pressure / 1000.0
  pub_baro.publish(msg_baro)
  


def sub_insCB(msg_in): 
  global pub_imu
  global pub_gps
  
  global msg_imu
  
  msg_imu.header.stamp          = msg_in.header.stamp
  msg_imu.header.frame_id       = msg_in.header.frame_id
  
  # Convert the RPY data from the Vectornav into radians!
  roll  = (math.pi * msg_in.RPY.x) / 180.0
  pitch = (math.pi * msg_in.RPY.y) / 180.0
  yaw   = (math.pi * msg_in.RPY.z) / 180.0
  q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  msg_imu.orientation.x = q[0]
  msg_imu.orientation.y = q[1]
  msg_imu.orientation.z = q[2]
  msg_imu.orientation.w = q[3]
         
  pub_imu.publish(msg_imu)
  
                                                 
  msg_gps                 = NavSatFix()
  msg_gps.header          = msg_in.header
  msg_gps.status.status   = NavSatStatus.STATUS_FIX # TODO - fix this
  msg_gps.status.service  = NavSatStatus.SERVICE_GPS
  msg_gps.latitude        = msg_in.LLA.x
  msg_gps.longitude       = msg_in.LLA.y
  msg_gps.altitude        = msg_in.LLA.z
  msg_gps.position_covariance_type  = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
  msg_gps.position_covariance[0]    = msg_in.PosUncerainty
  pub_gps.publish(msg_gps)
  
  
  msg_time = TimeReference()
  msg_time.header.stamp     = msg_in.header.stamp
  msg_time.header.frame_id  = msg_in.header.frame_id
  unix_time = 315964800 + (msg_in.Week * 7 * 24 * 3600) + msg_in.Time
  msg_time.time_ref = rospy.Time.from_sec(unix_time)
  pub_time.publish(msg_time)
  


if __name__ == '__main__':
  rospy.init_node('vectornav_sensor_msgs')
  
  global pub_imu
  global pub_mag
  global pub_temp
  global pub_baro
  global pub_gps
  global pub_time
  
  global msg_imu
    
  msg_imu = Imu()
  
  pub_imu  = rospy.Publisher("/Imu"          , Imu)
  pub_mag  = rospy.Publisher("/MagneticField", MagneticField)
  pub_temp = rospy.Publisher("/Temerature"   , Temperature)
  pub_baro = rospy.Publisher("/FluidPressure", FluidPressure)
  pub_gps  = rospy.Publisher("/NavSatFix"    , NavSatFix)
  pub_time = rospy.Publisher("/TimeRef"      , TimeReference)
  
  #TODO: Only subscribe when we have subscribers
  rospy.Subscriber("/vectornav/imu", sensors,  sub_imuCB)
  rospy.Subscriber("/vectornav/ins", ins,      sub_insCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
