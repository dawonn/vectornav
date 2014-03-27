#!/usr/bin/env python
#
# Dereck Wonnacott (C) 2013 
# MIT Licence (TODO: put license text here...)
# 
# Publishes an IMU message from the INS solution data
# 
# The INS solution only contains the orientation (YPR). If you also want the
# gyro and accel sensor data this script will need to be extend. This will slow down the
# update rate unless you are utilizing both device serial ports. ;)


from math import *
import time
import rospy
import tf
from vectornav.msg      import ins
from sensor_msgs.msg    import Imu

from tf.transformations import quaternion_from_euler
    

def subCB(msg_in):  
  global pub
    
  msg_out = Imu()
  msg_out.header = msg_in.header
  
  q = quaternion_from_euler(msg_in.RPY.x/180.0 * pi, 
                            msg_in.RPY.y/180.0 * pi, 
                            msg_in.RPY.z/180.0 * pi)
  
  msg_out.orientation.x = q[0] 
  msg_out.orientation.y = q[1]
  msg_out.orientation.z = q[2]
  msg_out.orientation.w = q[3]
  
  pub.publish(msg_out)
  


if __name__ == '__main__':
  rospy.init_node('ins2imu')
  
  global pub
  
  pub = rospy.Publisher("vectornav/imu/orient", Imu)

  rospy.Subscriber("vectornav/ins", ins, subCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
