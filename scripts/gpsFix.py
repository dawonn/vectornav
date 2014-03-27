#!/usr/bin/env python
#
# Dereck Wonnacott (C) 2013 
# MIT Licence (TODO: put license text here...)
# 
# This node publihses a NavSatFix from the INS solution data
#
 

import rospy
from vectornav.msg      import ins
from sensor_msgs.msg    import NavSatFix
from sensor_msgs.msg    import NavSatStatus

import time

def subCB(msg_in):  
  global pub
                                                 
  msg_out = NavSatFix()
  msg_out.header = msg_in.header
  
  msg_out.status.status  = NavSatStatus.STATUS_FIX # TODO - fix this
  msg_out.status.service = NavSatStatus.SERVICE_GPS

  msg_out.latitude   = msg_in.LLA.x
  msg_out.longitude  = msg_in.LLA.y
  msg_out.altitude   = msg_in.LLA.z
  
  msg_out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
  msg_out.position_covariance[1] = msg_in.PosUncerainty
  
  pub.publish(msg_out)
  


if __name__ == '__main__':
  rospy.init_node('ins2NavSatFix')
  
  global pub
  
  pub = rospy.Publisher("vectornav/ins/NavSatFix", NavSatFix)
  

  rospy.Subscriber("vectornav/ins", ins, subCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
