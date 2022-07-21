# rtklib_ros_bridge
[![CircleCI](https://circleci.com/gh/MapIV/rtklib_ros_bridge.svg?style=svg&circle-token=b0ded687015c8cd5440cd1436a31890c3a1697c4)](https://circleci.com/gh/MapIV/rtklib_ros_bridge)
## Overview

rtklib_ros_bridge is a package that outputs the latitude and longitude, satellite reception status, altitude, ecef xyz, ecef velocity vector, and Time of Week (GPS Time) calculated by RTKLIB as ROS messages.

## Install

1) First, download the modified RTKLIB to your home directory.

		cd $HOME  
		git clone https://github.com/MapIV/RTKLIB.git
		cd $HOME/RTKLIB     
		git checkout rtklib_ros_bridge_b34

	[About RTKLIB](http://www.rtklib.com)

2) Build RTKLIB.
  
		cd $HOME/RTKLIB/lib/iers/gcc/  
		make   
		cd $HOME/RTKLIB/app  
		make   

3) Change the permissions of the two files.
 
		cd $HOME/RTKLIB/app/rtkrcv/gcc  
		chmod 755 rtkstart.sh  
		chmod 755 rtkshut.sh  

4) Next, download and build rtklib_ros_bridge.

		cd $HOME/colcon_ws/src  
		git clone -b ros2-v0.1.0 https://github.com/MapIV/rtklib_ros_bridge.git  
		cd ..  
		colcon build  

## Configuration
1) Open RTKLIB settings.

		gedit $HOME/RTKLIB/app/rtkrcv/conf/rtklib_ros_bridge_sample.conf

2) Set the serial device on line 10. If you connect using USB, it is OK.

>Line 10:  
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:9600:8:n:1:off  

※If you know the device number "/dev/ttyACM-" but OK.

3) Next, configure the receiver from ublox application, u-center.The usage of u-center is not described here. Below is an overview of the settings. (Here is how to use a Ublox receiver)  

* Enable UBX message ※Set to output only RAWX and SFRBX
* Save your settings last.

	[About u-center](https://www.u-blox.com/product/u-center)  

## Usage
1) Connect the GNSS receiver and start RTKLIB.

		cd $HOME/RTKLIB  
		bash rtklib_ros_bridge_single.sh  

2) Check the status of RTKLIB. If GPS Time is moving, it is OK. Execute the following command in the terminal of item 3.

		status 0.1  

※If GPS Time is not working, there may be a mistake in the receiver settings or RTKLIB settings.

3) Start rtklib_ros_bridge.

		cd $HOME/colcon_ws/src 
		source install/setup.bash
		ros2 launch rtklib_bridge rtklib_bridge.launch.xml
		
