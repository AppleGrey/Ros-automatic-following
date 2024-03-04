/*
	DBSCAN Algorithm
	15S103182
	Ethan
*/
#pragma once
#ifndef LIDARTRACKER_H_ 

#define LIDARTRACKER_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

using namespace std;
class point {
public:
	float x;
	float y;
	float angle;
	// int cluster = 0;
	// int pointType = 1;//1 noise 2 border 3 core
	// int pts = 0;//points in MinPts 
	// vector<int> corepts;
	// int visited = 0;
	// point() {}
	// point(float a, float b, int c) {
	// 	x = a;
	// 	y = b;
	// 	cluster = c;
	// }
};




class LidarTracker{
public:
    void init(cv::Rect r,std::vector<float> lid);//雷达追踪初始化
    cv::Rect lidarForsee(std::vector<float> lid,cv::Mat& m);//雷达预测
	cv::Rect lidarForsee_2(std::vector<float> lid,std::vector<float> loc);//雷达预测2
	float getConf();
	void setParam(double minD){md = minD;};

private:
	cv::RNG rng;
	const int stateNum = 4;                                      //状态值4×1向量(x,y,△x,△y)
	const int measureNum = 2;                                    //测量值2×1向量(x,y)	
	cv::KalmanFilter KF;
	cv::Mat measurement;

	//最低距离(m)
	float md=0.08;

	//锁定目标
	float nx;
	float ny;


    float a_angle;
    float a_distance;
	int a_features=0;

	double cal_angle(double k);
	double cal_angle_f(double x,double y);
	float squareDistance(point a, point b);
	vector<point> DBSCAN(vector<point> dataset, float Eps, int MinPts);
	// bool _cmp(point p1,point p2);

	vector<point> classify(vector<point> dataset);//雷达点云分类切割

	const double MIN_DIS = 0.1;
	const double MAX_DIS = 5.0;

	const double PAR_LEN = 180.0;
	const double VER_LEN = 121.0;

	vector<point> lidarStandard;
	float conf=0;
	float maxD=3;


	float xmax=2;
	float ymax=1;
};
