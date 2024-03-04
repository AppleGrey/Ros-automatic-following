#include"Lidar.h"

fw::Lidar::Lidar()
{
	lidar_sub = nh_.subscribe("/scan", 1, &fw::Lidar::LidarCb, this);
}

void fw::Lidar::LidarCb(const sensor_msgs::LaserScan scan_msg)
{
    // std::cout<<"收到lidar msg"<<std::endl;
    lidarQueue.push(scan_msg.ranges);
    if (lidarQueue.size() <= DelayNum+2) {
        return;
    }
    lidarQueue.pop();
}

std::vector<float> fw::Lidar::getMsg()
{
    const double MIN_DIS = 0.2;
	const double MAX_DIS = 3.0;
    std::vector<float> ret;
    // std::cout<<lidarQueue.size()<<std::endl;
    if(lidarQueue.size()>1){
        for(int i=0;i<720;i++){
            if(lidarQueue.front()[i] > MIN_DIS && lidarQueue.front()[i] < MAX_DIS) {
                ret.push_back(lidarQueue.front()[i]);
            }
            else{
                ret.push_back(10);
            }
        } 
    }
    return ret;
}

