#!/usr/bin/env python
#
# Subscribe to gps_common's odom and publish our own ;)
# Dereck Wonnacott (2013) - MIT Licence (TODO: put license text here?)
#  

import rospy
from nav_msgs.msg    import Odometry

from copy            import deepcopy

global first_msg 

def subCB(msg_in):  
  global pub
  global first_msg
  
  # I want to provide odom in local coords for now.
  if first_msg == "":
    first_msg = deepcopy(msg_in)
    
  #print first_msg
                                    
  msg_out = Odometry()
  msg_out = msg_in
  
  msg_out.header.stamp = rospy.Time.now()

  msg_out.pose.pose.position.x -= first_msg.pose.pose.position.x
  msg_out.pose.pose.position.y -= first_msg.pose.pose.position.y
  msg_out.pose.pose.position.z -= first_msg.pose.pose.position.z
  
  pub.publish(msg_out)
  


if __name__ == '__main__':
  rospy.init_node('local_odom')
  
  global pub
  global first_msg
  
  first_msg = ""
  
  pub = rospy.Publisher("/odom/local", Odometry)
  
  rospy.Subscriber("/odom", Odometry, subCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
