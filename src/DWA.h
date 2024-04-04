//
// Created by chh3213 on 2022/11/26.
//

#ifndef CHHROBOTICS_CPP_DWA_H
#define CHHROBOTICS_CPP_DWA_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

#define PI 3.141592653589793238

class DWA {
private:
    double dt=0.1; //采样时间
    double v_min=-0.5,v_max=0.3,w_min=-40*PI/180,w_max=40*PI/180; //线速度角速度边界
    double predict_time=0.7;//轨迹推算时间长度
    double a_vmax=0.4,a_wmax=160*PI/180; //线加速度和角加速度最大值
    double v_sample=0.01,w_sample=0.1*PI/180; //采样分辨率
    double alpha=0.5,beta=2.0,gamma=1.0,epsilon=1.0,zeta=1.0,eta=1.0; //轨迹评价函数系数
    double w_heading=1.0,w_velocity=1.0,w_distance=1.0,w_goal_distance=1.0,w_smooth=1.0,w_path_overlap=1.0; //轨迹评价函数权重
    double radius=0.5; // 用于判断是否到达目标点（应该是小车的半径）
    double judge_distance=10; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    double L=0.18; //前后轮距
    
    vector<VectorXd> states; //存储历史状态
    vector<Vector2d> goals; //存储历史目标点
    const int MAX_STORGE=600; //最大存储历史状态数

    double min_path_distance=0.5; //路径重合度阈值
private:
    vector<double> calVelLimit();
    vector<double> calAccelLimit(double v, double w);
    vector<double> calObstacleLimit(VectorXd state, const vector<Vector2d> &obstacle);
    vector<double> calDynamicWindowVel(double v, double w,VectorXd state, const vector<Vector2d>& obstacle);
    double _dist(VectorXd state, const vector<Vector2d>& obstacle);
    vector<VectorXd> trajectoryPredict(VectorXd state, double v, double w);
    pair<vector<double>,vector<VectorXd>> trajectoryEvaluation(VectorXd state, Vector2d goal, const vector<Vector2d>& obstacle);

    double _heading(const vector<VectorXd>& trajectory,Vector2d goal);
    double _velocity(const vector<VectorXd>& trajectory);
    double _distance(const vector<VectorXd>& trajectory, const vector<Vector2d>& obstacle);
    double _goalDist(const vector<VectorXd>& trajectory, Vector2d goal);
    double _smooth(const Vector2d speeds);
    double _pathOverlap(const vector<VectorXd>& trajectory);


public:
    DWA(double dt, double vMin, double vMax, double wMin, double wMax, double predictTime, double aVmax, double aWmax,
        double vSample, double wSample, double alpha, double beta, double gamma, double radius, double judgeDistance);

    DWA();

    VectorXd kinematicModel(VectorXd state, vector<double>control,double dt);

    pair<vector<double>,vector<VectorXd>> dwaControl(VectorXd state, Vector2d goal, const vector<Vector2d> &obstacle);

};


#endif //CHHROBOTICS_CPP_DWA_H
