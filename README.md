# Vectornav ROS2 Driver

A ROS2 node for VectorNav INS & GPS devices.

This package provides a sensor_msg interface for the VN100, 200, & 300 
devices. Simply configure your launch files to point to the serial port
of the sensor and you can use ros2 topic to quickly get running. 

This driver is designed to get you running quickly, but you will likely want 
to customize main.cc to change device parameters for your application.

## QuickStart Guide

Build

1. git clone https://github.com/dawonn/vectornav.git -b foxy
2. cd vectornav 
3. colcon build

Run

4. (Terminal 1) roslaunch vectornav vectornav.launch
5. (Terminal 2) rostopic echo /vectornav/IMU

## vectornav node

This node provides a ROS2 interface for a vectornav device. It can be configured
via ROS parameters and publishes sensor data via ROS topics.


## vectronav.launch

This launch file contains the default parameters for connecting a device to ROS.
You will probably want to copy it into your own project and modify as required. 


## References 

[1] http://www.vectornav.com/ "VectorNav"
