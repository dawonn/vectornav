#!/usr/bin/env python
#
# Hokuyo lidar orientation from IMU
#  

import rospy
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion

from   geometry_msgs.msg   import TransformStamped
from   geometry_msgs.msg   import Vector3
from   vectornav.msg       import ins
from   vectornav.msg       import sensors


def insCB(msg_in):
  global pub_tf
  global msg_tf   

  msg_tf.header.stamp = rospy.Time.now()

  
  quaternion = tf.transformations.quaternion_from_euler( msg_in.RPY.x/180.0 * 3.1415,
                                                        -msg_in.RPY.y/180.0 * 3.1415,
                                                        -msg_in.RPY.z/180.0 * 3.1415)
  msg_tf.transform.rotation.w = quaternion[3]
  msg_tf.transform.rotation.x = quaternion[0]
  msg_tf.transform.rotation.y = quaternion[1]
  msg_tf.transform.rotation.z = quaternion[2]
  
  pub_tf.sendTransform(msg_tf)


def imuCB(msg_in):
  global msg_tf 
  global vel
  global last_msg_stamp

  # Calculate the time since the last sensor message
  if(last_msg_stamp == 0):
    last_msg_stamp = msg_in.header.stamp
    return
  dt = last_msg_stamp - msg_in.header.stamp
  
  # Update estimated velocity
  #vel.x += msg_in.Accel.x * dt.to_sec()
  #vel.y += msg_in.Accel.y * dt.to_sec()
  #vel.z += (msg_in.Accel.z + 9.45162) * dt.to_sec()
  
  print msg_in.Accel.x, msg_in.Accel.y, msg_in.Accel.z, vel.x, vel.y, vel.z
  
  # Update estimated pose
  msg_tf.transform.translation.x += vel.x * dt.to_sec()
  msg_tf.transform.translation.y += vel.y * dt.to_sec()
  msg_tf.transform.translation.z += vel.z * dt.to_sec()
  
  last_msg_stamp = msg_in.header.stamp



if __name__ == '__main__':
  global pub_tf   
  global msg_tf
  global vel
  global last_msg_stamp
  
  rospy.init_node('vn_orient')
  
  msg_tf = TransformStamped()
  msg_tf.header.frame_id = "laser_base"
  msg_tf.child_frame_id  = "laser"
  pub_tf = tf2_ros.TransformBroadcaster()
  
  
  msg_tf.transform.translation.x = 0
  msg_tf.transform.translation.y = 0
  msg_tf.transform.translation.z = 0
  vel = Vector3()
  vel.x = 0
  vel.y = 0
  vel.z = 0
  last_msg_stamp    = 0
  
  rospy.Subscriber("vectornav/ins", ins, insCB)
  #rospy.Subscriber("vectornav/imu", sensors, imuCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
