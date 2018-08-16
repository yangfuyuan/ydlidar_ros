YDLIDAR ROS PACKAGE V1.3.7
=====================================================================

ROS node and test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

How to build YDLIDAR ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    --$ git clone https://github.com/yangfuyuan/ydlidar_ros
    --$ cd ydlidar_ros/ydlidar_sdk
    --$ git submodule init
    --$ git submodule update

    2) Running catkin_make to build ydlidar_node and ydlidar_client
    3) Create the name "/dev/ydlidar" for YDLIDAR
    --$ roscd ydlidar/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh

How to run YDLIDAR ros package
=====================================================================
There're two ways to run YDLIDAR ros package

1. Run YDLIDAR node and view in the rviz
------------------------------------------------------------
roslaunch ydlidar lidar_view.launch

You should see YDLIDAR's scan result in the rviz.

2. Run YDLIDAR node and view using test application
------------------------------------------------------------
roslaunch ydlidar lidar.launch

rosrun ydlidar ydlidar_client

You should see YDLIDAR's scan result in the console


Parameters
=====================================================================
port (string, default: /dev/ydlidar)

    serial port name used in your system. 

baudrate (int, default: 115200)

    serial port baud rate. 

frame_id (string, default: laser_frame)

    frame ID for the device. 

low_exposure (low_exposure, default: false)

    indicated whether the LIDAR has low light power mode. 

heartbeat (bool, default: false)

    indicated whether the LIDAR IS powered off. 

resolution_fixed (bool, default: true)

    indicated whether the LIDAR has a fixed angular resolution. 

auto_reconnect (bool, default: true)

    indicated whether the LIDAR auto reconnection. 

reversion (bool, default: false)

    indicated whether the LIDAR data rotation 180°. 

intensity (bool, default: false)

    indicated whether the LIDAR has intensity. 

angle_min (double, default: -180)

    Min valid angle (°) for LIDAR data. 

angle_max (double, default: 180)

    Max valid angle (°) for LIDAR data. 

range_min (double, default: 0.08)

    Min valid range (m) for LIDAR data. 

range_max (double, default: 16.0)

    Max valid range (m) for LIDAR data. 

ignore_array (string, default: "")

    Set the current angle range value to zero. 

samp_rate (int, default: 4)

    the LIDAR sampling frequency.

frequency (double, default: 7)

    the LIDAR scanning frequency.



Upgrade Log
=====================================================================

2018-08-16 version:1.3.7

   1.current SDK verison 1.3.7.

   2.update SDK interface.

   3.Check if the Lidar port setting is correct.

   4.add ydldiar_sdk submodule
 






