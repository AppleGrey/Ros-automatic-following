#ifndef MY_BRIDGE_H
#define MY_BRIDGE_H

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <memory>

#endif

class Cv_Bridge{
public:
    std::shared_ptr<sensor_msgs::Image> CvToRos(cv::Mat& img);
    std::shared_ptr<cv::Mat> RosToCv(const sensor_msgs::Image::ConstPtr& img);
private:
    int GetType(std::string type);
    std::string GetType(int type);
    int PixelSize(int type);
};



