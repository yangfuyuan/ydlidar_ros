/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "LIDARDevice.h"
//#include <config.h>
#include <vector>
#include <iostream>
#include <string>

using namespace ydlidar;

#define ROSVerision "1.3.7"

ros::Publisher scan_pub;
std::string frame_id;

std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


void LaserScanCallback(const LaserScan& scan) {

    sensor_msgs::LaserScan scan_msg;
    ros::Time start_scan_time;    
    start_scan_time.sec = scan.self_time_stamp/1000000000ul;
    start_scan_time.nsec = scan.self_time_stamp%1000000000ul;
    scan_msg.header.stamp = start_scan_time;
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = scan.config.min_angle;
    scan_msg.angle_max = scan.config.max_angle;
    scan_msg.angle_increment = scan.config.ang_increment;
    scan_msg.scan_time = scan.config.scan_time;
    scan_msg.time_increment = scan.config.time_increment;
    scan_msg.range_min = scan.config.min_range;
    scan_msg.range_max = scan.config.max_range;   
    scan_msg.ranges = scan.ranges;
    scan_msg.intensities =  scan.intensities;
    scan_pub.publish(scan_msg);

}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "ydlidar_node"); 

    std::string port;
    int baudrate=115200;
    bool intensities,low_exposure,reversion, resolution_fixed,heartbeat;
    bool auto_reconnect;
    double angle_max,angle_min;
    int samp_rate;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range , min_range;
    double _frequency;

    ros::NodeHandle nh;
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); 
    nh_private.param<int>("baudrate", baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    nh_private.param<bool>("heartbeat", heartbeat, "false");
    nh_private.param<bool>("low_exposure", low_exposure, "false");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<bool>("reversion", reversion, "false");
    nh_private.param<bool>("intensity", intensities, "false");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<int>("samp_rate", samp_rate, 4); 
    nh_private.param<double>("range_max", max_range , 16.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<double>("frequency", _frequency , 7.0);

    nh_private.param<std::string>("ignore_array",list,"");
    ignore_array = split(list ,',');

    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between -180 and 180");
        }
    }

    LaserParamCfg cfg;

    cfg.serialPort = port;
    if(cfg.serialPort.empty()) {
        ROS_ERROR_STREAM("ignore array is odd need be even");
        return 0;
                
    }

        
    if(_frequency<5){
       _frequency = 7.0; 
    }
    if(_frequency>12){
        _frequency = 12;
    }
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    cfg.ignoreArray = ignore_array;

    cfg.serialBaudrate = baudrate;
    cfg.sampleRate = samp_rate;
    cfg.scanFrequency = _frequency;

    cfg.intensity = intensities;
    cfg.autoReconnect = auto_reconnect;
    cfg.exposure = low_exposure;
    cfg.fixedResolution = resolution_fixed;
    cfg.reversion = reversion;
    cfg.heartBeat = heartbeat;

    cfg.maxAngle = angle_max;
    cfg.minAngle = angle_min;
    cfg.maxRange = max_range;
    cfg.minRange = min_range;

    ros::Rate rate(30);
    printf("[YDLIDAR INFO] Current ROS Driver Version: %s\n",((std::string)ROSVerision).c_str());
    //printf("[YDLIDAR INFO] SDK Version: %s\n", ((std::string)SDK_VERSION.c_str());
    //printf("[YDLIDAR INFO] LIDAR Version: %s\n", ((std::string)YDLIDAR_VERSION.c_str());


    try {

        LIDAR ydlidar;

        std::vector<string> ports =  ydlidar.getLidarList();
        for(std::vector<string>::iterator it = ports.begin(); it != ports.end(); it++) {
            printf("Available radar: %s\n", (*it).c_str());
        }
        ydlidar.RegisterLIDARDataCallback(&LaserScanCallback);
        ydlidar.UpdateLidarParamCfg(cfg);

         while(ros::ok()){
             try {
                 ydlidar.spinOnce();
             }catch(TimeoutException& e) {
                 std::cout<< e.what()<<std::endl;

             }catch(CorruptedDataException& e) {
                 std::cout<< e.what()<<std::endl;

             }catch(DeviceException& e) {
                 std::cerr<< e.what()<<std::endl;
                 break;
             }
             rate.sleep();
             ros::spinOnce();
         }


    }catch(TimeoutException& e) {
        std::cout<< e.what()<<std::endl;
    }catch(CorruptedDataException& e) {
        std::cout<< e.what()<<std::endl;
    }catch(DeviceException& e) {
        std::cerr<< e.what()<<std::endl;
    }

    return 0;
}
