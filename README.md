Vectornav ROS Driver
====================

A ROS node for `VectorNav` INS & GPS devices.

This package provides a sensor_msg interface for the VN100, 200, & 300 
devices. Simply configure your launch files to point to the serial port
of the deice and you can use rostopic to quickly get running.   

Check out the ROS2 branch for ROS2 Support!


QuickStart Guide
----------------

This assumes that you have a VectorNav device connected to your computer 
via a USB cable and that you have already created a `[catkin workspace]`[2]

Build:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/dawonn/vectornav.git
$ cd ..
$ catkin_make
```

Run:

```bash
(Terminal 1) $ roscore
(Terminal 2) $ roslaunch vectornav vectornav.launch
(Terminal 3) $ rostopic list
(Terminal 3) $ rostopic echo /vectornav/IMU
(Terminal #) $ ctrl+c to quit
```


Overview 
--------

#### vnpub node

This node provides a ROS interface for a vectornav device. It can be configured
via ROS parameters and publishes sensor data via ROS topics.


#### vectornav.launch

This launch file contains the default parameters for connecting a device to ROS.
You will problaby want to copy it into your own project and modify as required. 


References 
----------

```
[1]: http://www.vectornav.com/ "VectorNav"
[2]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment "ROS Workspace Tutorial"
```

The MIT License (MIT)
----------------------
```

Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>

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

```

