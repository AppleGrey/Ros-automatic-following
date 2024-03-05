/*
//	Name: CTracking
//	Version: v1.0
*/


#pragma once
#ifndef CTRACKING_H_ 

#define CTRACKING_H_

#include<iostream>
#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Lidar.h"
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "fdssttracker.hpp"
#include "lidartracker.h"
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include "grubbs.hpp"
#include <thread>
#include "my_bridge.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include "DWA.h"

#endif


// #define Min_distance 0.5
// #define Max_distance 3.0

class CTracking {
public:
	CTracking(int cid);//初始化(信号传输)
	void init();//初始化(追踪)

	void setParam(double minV,double maxV,double D,double minD){Min_distance=D;Min_linear_speed=minV;Max_linear_speed=maxV;lidarTracker->setParam(minD);};

	cv::Rect selectROI();//选框
	cv::Rect selectROI(int l1,int l2,int r1,int r2);//远程锁定

	bool lock(cv::Rect rect);//锁定

	void run();//开车
	void stop();//停车
	

	

private:
	bool isStart=false;

	double MIN_DIS = 0.0;
	double MAX_DIS = 8.0;

	const double PAR_LEN = 180.0;
	const double VER_LEN = 121.0;

	double Max_linear_speed = 0.8;
	double Min_linear_speed = 0.2;
	double Min_distance = 0.3;
	const double Max_distance = 3.0;
	const double Max_rotation_speed = 0.75;

	float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
	float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

	const double L=0.18;

	
	float vmin=0.3;
	float lmin=0.7;


	cv::VideoCapture* capture;//
	fw::Lidar* lidar;//

	FDSSTTracker* visionTracker;//fDSST
	LidarTracker* lidarTracker;//lidar

	cv::Mat firstImg;//����֡
	std::vector<float> firstLid;


	cv::Rect visionForsee(cv::Mat input);//视觉预测
	// std::queue<sensor_msgs::LaserScan::_ranges_type> temp getLidar();//
	cv::Rect lidarForsee(std::vector<float>,cv::Mat&);//雷达预测

	cv::Rect fusion(cv::Rect r1, cv::Rect r2);//

	
	/** 
	* @brief calculate linear and rotation speed
	* @param rect  position of target
	*
	* @return the first value is linear speed and the second is rotation speed
	*/
	std::pair<double, double> cal_speed(double x,double w,double distance);
	std::pair<double, double> cal_vel(double distance, double angle);

	//calculate angle of the target by camera data
	double cal_angle(double);

	//calculate distance of the target
	double cal_distance(double min_angle, double max_angle,const vector<float> &lid);

	bool nav();//路径规划
	bool move();//移动


	std::thread* t;

	ros::NodeHandle* nh_r;
	ros::Subscriber sub;

	// void t_send();

	//kalman滤波
	cv::RNG rng;
	const int stateNum = 16;                                      //状态值16×1向量(x,y,△x,△y)
	const int measureNum = 8;                                    //测量值8×1向量(x,y)	
	cv::KalmanFilter KF;
	cv::Mat measurement;

	cv::Mat visionStandard;//用于视觉评定
};