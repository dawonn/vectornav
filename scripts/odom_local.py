#!/usr/bin/env python
#
# Dereck Wonnacott (C) 2013 
# MIT Licence (TODO: put license text here...)
# 
# This node takes in an Odometry messages and filters it out to the local cood
# frame since the node was started. (Filters out huge global offsets in GPS data)
#

import rospy
import tf
from math import *

from copy import deepcopy
from nav_msgs.msg    import Odometry
from geometry_msgs.msg import Quaternion

global first_msg 

def subCB(msg_in):  
  global pub
  global first_msg
  
  # Save the first odom message as the 'reference' frame
  if first_msg == "":
    first_msg = deepcopy(msg_in)
    return
                                    
  msg_out = Odometry()
  msg_out = msg_in

  # Only publish changes from the intial reference frame
  msg_out.pose.pose.position.x -= first_msg.pose.pose.position.x
  msg_out.pose.pose.position.y -= first_msg.pose.pose.position.y
  msg_out.pose.pose.position.z -= first_msg.pose.pose.position.z
  
  # I need to learn Q-math again....
  xyzw_array = lambda o: [o.x, o.y, o.z, o.w]
  rpy_first = xyzw_array(first_msg.pose.pose.orientation)
  rpy_out = xyzw_array(msg_in.pose.pose.orientation)
                                               
  
  q = tf.transformations.quaternion_from_euler(rpy_out[0] - rpy_first[0], 
                                               rpy_out[1] - rpy_first[1],
                                               rpy_out[2] - rpy_first[2] )
  msg_out.pose.pose.orientation = Quaternion(*q)
  
  pub.publish(msg_out)
  


if __name__ == '__main__':
  rospy.init_node('local_odom')
  
  global pub
  global first_msg
  
  first_msg = ""
  
  pub = rospy.Publisher("/vectornav/ins/odom/local", Odometry)
  
  rospy.Subscriber("/vectornav/ins/odom", Odometry, subCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
