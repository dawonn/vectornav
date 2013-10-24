Vectornav ROS Driver
====================

A ROS node for VectorNav INS & GPS devices.

This package is still under development but currently is able to 
connect to a [VN-200][2] and poll each of the three data packets:
{gps, ins, sensors} and publish the data as ROS topics. 



The MIT License (MIT)
----------------------

Copyright (c) 2013 Dereck Wonnacott <dereck@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.



QuickStart Guide
----------------

This assumes that you have a VN-200 device connected to your computer 
via a USB cable and that you have already created a [catkin workspace][3]

Build:

1. cd ~/catkin_ws/src
2. git clone https://github.com/dawonn/vectornav.git
3. cd ..
4. catkin_make

Run:

5. (Terminal 1) roscore
6. (Terminal 2) roslaunch vectornav vn200.launch
7. (Terminal 3) rostopic list
8. (Terminal 3) rostopic echo /vectornav/imu
9. (Terminal #) ctrl+c to quit



Overview 
--------

### Nodes

#### src/vn200_node

This node provides a ROS interface for the VN-200 device. It can be configured
via ROS parameters and pubishes sensor and solution data via ROS topics.

#### scripts/vn_sensor_msgs.py

This node converts the raw device specific messages into ROS standard messages.

Subscribes:
* vectornav::ins
* vectornav::sensors

Publishers:
* sensor_msgs::Imu
* sensor_msgs::MagneticField
* sensor_msgs::Temperature
* sensor_msgs::FluidPressure
* sensor_msgs::NavSatFix
* sensor_msgs::NavSatStatus
* sensor_msgs::TimeReference

Note: if you're application can avoid subscribing to both device topics, the data
rate can be approximatly doubled. :)

#### scripts/local_odom.py

This node subsribes to an Odometry topic and republishes it with the origin
located at position where the node is started. Intended to be used with GPS-based
odometry. 

### Launch Files

#### vn200.launch

This launch file contains the default parameters for connecting a VN-200 
device to ROS. 

If you want standard ROS topics you should also run vn_sensor_msgs.py. 

#### odom.launch

Attempts to generate odometry from the GPS. Don't trust it though~ ;)
In my simple experiments I had an error of 30 meters.



Device Notes
------------

### Firmware
As of today (Oct 22, 2013) the VN-200 firmware is still in an alpha state.
It cannot yet perform most of the features that are advertised in the 
datasheets and product specifications. The beta version of the firmware 
was due in early October, but is still stuck in their internal QA process. 
This means that the VN-200 is basilcy a simple sensor, processing needs
to be done externally until the new firmware is released.

### Available Data
Lets touch on what we can do for now though. :) The device has three types
of data packets which can be accessed:

1. INS - Basic sensor fusion between GPS and IMU.
2. GPS - Raw GPS data
3. Sensors - _Calibrated_ Sensor data

### Device Ports
The VN-200 has two serial ports on the device. The development kit ships with
two cables, each cable only allows access to one of the ports but you cannot 
use more than one cable at a time. If you want to use both ports, you need to
build a custom cable. I have built such a cable but not yet been able to utilize
the 5 Volt port yet for some reason. Mouser Part Numbers:

* 855-M80-4861005
* 895-TTL-232RGVSW5VWE   (Probably the wrong adapter)
* 895-TTL-232RGVSW3V3WE  (Work fine)

### Device Connections
You may either poll for data over each of the ports or set up 
asyncronous messaging at a fixed rate. Each port can only utilize one type of 
asyncronous data packet at a time. Polling is the only method for getting all 
of the data from the device. 

### VectorNav Device Library
VectorNav ships a c/c++ libaray for communicating with the device. I stripped
out the unnessiary binary blobs and repacked it within this node so this node can
be self-contained. 

### Future Firmware
Based on talks with the engineers at VectorNav, the future is bright! 

Soon ("Early October 2013") the new firmware will be released and finally 
provide all of the specifications that they have been advertising. The device 
will be able to do on-board motion processing and a whole bunch-o-stuff. The most
important new feature in my mind is a new binary message protocol. The current 
firmware sends all data over the wire in ASCII, which is slow. The new 
firmware should allow much faster update rates. Better yet, we will be able
to tell the device what data to send over the wire with higher fiedelity by
setting a register in the deivce rather than being forced to choose from the 
three data packets.



Implementation Notes
--------------------

The current node is the result of messing around the the device to see what its
capailbities are. As such, it's rather messy and only really supports polling
right now. You can enable async, but it will only print the data to the screen and
not publish any topics. It's pretty easy to add, if you want me to do so, just ask. 
When the new firmware comes out, the driver will need to be rewritten though.

I set up the node so that it only polls data for the topics that have been
subscribe to so that you can maximize the polling rate. I have been able to poll
sensors at 100Hz for example, but it degrades to 40Hz with both Sensor and INS data.

The node publishes a device specific message type for each of the packet types. If 
you want to use standard ROS messages, it is trivial to write a conversion node in 
python to read the device messages and publish the standard message. I have a few 
examples in the _scripts_ directory. This keeps the device node's code lean and 
allows you to maximize the data rate based on your needs. 

The killer example is the *sensor_msgs::Imu* message. To properly fill
the message fields would require a both the INS (for orientation) and Sensors data 
packets. This limits you to 40Hz because it requires both data types. If your 
application only requires orientation (such are robot_pose_ekf users) then
you can publish a message that only subscribes to the INS messages and get much better
update rates. Alternativly, if you want to write your own orientation filter you can 
ignore the INS messages and subscribe only to the Sensor packets.



References 
----------

[1]: http://www.vectornav.com/ "VectorNav"
[2]: http://www.vectornav.com/products/vn100-smt?id=48 "VN-200"
[3]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment "ROS Workspace Tutorial"




