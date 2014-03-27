#!/usr/bin/env python
#
# Dereck Wonnacott (C) 2013 
# MIT Licence (TODO: put license text here...)
# 
# Publishes an Odometry message from the INS solution data
#


from math import *

import rospy
import tf
from geometry_msgs.msg import Quaternion
from vectornav.msg   import ins
from nav_msgs.msg    import Odometry


def LLA2ECEF(lat, lon, alt):
    # see: http://www.mathworks.de/help/toolbox/aeroblks/llatoecefposition.html
    rad = 6378137.0                      # radius     (WGS84)
    f   = 1.0/298.257223563              # flattening (WGS84)
    ls  = atan((1 - f)**2 * tan(lat))    # lambda

    x = rad * cos(ls) * cos(lon) + alt * cos(lat) * cos(lon)
    y = rad * cos(ls) * sin(lon) + alt * cos(lat) * sin(lon)
    z = rad * sin(ls) + alt * sin(lat)

    return [x, y, z]
    
def LLA2Naive(lat, lon, alt):
    x = lon * 50000.0
    y = lat * 100000.0
    z = alt

    return [x, y, z]
    

def subCB(msg_in):  
  global pub
  
  msg_out = Odometry()
  msg_out.header = msg_in.header
  msg_out.header.frame_id = "Pioneer3AT/odom"
  msg_out.child_frame_id = "imu"
  
  cart = LLA2Naive(msg_in.LLA.x, msg_in.LLA.y, msg_in.LLA.z)
  
  
  msg_out.pose.pose.position.x = cart[0]
  msg_out.pose.pose.position.y = cart[1]
  msg_out.pose.pose.position.z = cart[2]
  
  q = tf.transformations.quaternion_from_euler(pi * msg_in.RPY.x /  180.0, 
                                               pi * msg_in.RPY.y /  180.0,
                                               pi * msg_in.RPY.z / -180.0 )
  msg_out.pose.pose.orientation = Quaternion(*q)
                                                                           
  pub.publish(msg_out)
  


if __name__ == '__main__':
  rospy.init_node('ins2Odometry')
  
  global pub
  
  pub = rospy.Publisher("vectornav/ins/odom", Odometry)
  

  rospy.Subscriber("vectornav/ins", ins, subCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
