#pragma once
#ifndef LIDAR_H_ 

#define LIDAR_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <cmath>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>

 #endif
// #include"our_track/example.h"

// #define Max_linear_speed 0.8
// #define Min_linear_speed 0.4
// #define Min_distance 0.5
// #define Max_distance 3.0
// #define Max_rotation_speed 0.75


namespace fw {
	class Lidar {
	public:
		Lidar();
		bool isValid();

		void LidarCb(const sensor_msgs::LaserScan scan_msg);
		
		std::vector<float> getMsg();


	private:
		ros::NodeHandle nh_;
		ros::Subscriber lidar_sub;

		const int DelayNum = 0;
		// sensor_msgs::LaserScan::_ranges_type msg;
		std::queue<sensor_msgs::LaserScan::_ranges_type> lidarQueue;
	};
}

