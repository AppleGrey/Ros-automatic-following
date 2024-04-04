#include "lidartracker.h"
#include <random>

void LidarTracker::init(cv::Rect r,std::vector<float> lid){
    //数据处理
    vector<point> dataset;
    int a=0;
    
    for(float& f : lid){
        point p;
        p.x=sin(((float)a/2)/180*M_PI)*f;
        p.y=cos(((float)a/2)/180*M_PI)*f;
        p.angle=(float)a/2;
        dataset.push_back(p);
        std::cout<<"x:"<<p.x<<" y:"<<p.y<<std::endl;
        a++;
    }
    
    //聚类(分类)
    vector<vector<point>> res;
    vector<point>* temp=new vector<point>;
    temp->push_back(dataset[0]);
    for(int i=1;i<dataset.size();i++){
        if(pow(dataset[i-1].x-dataset[i].x,2)+pow(dataset[i-1].y-dataset[i].y,2)<=pow(md,2)){
            temp->push_back(dataset[i]);
        }
        else{
            res.push_back(*temp);
            temp=new vector<point>;
            temp->push_back(dataset[i]);
        }
        if(i==719){
            if(pow(dataset[i].x-dataset[0].x,2)+pow(dataset[i-1].y-dataset[0].y,2)<=pow(md,2)){
                for(int j=0;j<res[0].size();j++){
                    temp->push_back(res[0][j]);
                }
                res.erase(res.begin());
                res.push_back(*temp);
            }
            else{
                res.push_back(*temp);
            }
        }
    }

    //测试显示
    std::default_random_engine generator;
    int minColor = 0;
    int maxColor = 255;
    cv::Mat img(1000,1000,16,cv::Scalar(0,0,0));
    for(vector<point>& ps:res){
        cv::Scalar randomColor(generator() % (maxColor - minColor + 1) + minColor,
                           generator() % (maxColor - minColor + 1) + minColor,
                           generator() % (maxColor - minColor + 1) + minColor);
        for(point p : ps){
            cv::Point p2(p.x*100+400,p.y*100+400);
            cv::circle(img,p2,1,randomColor,-1);
        }
    }
    

    //初始位置
    double scale = ((double)r.x+r.width/2)/640;
    std::cout<<"scale:"<<scale<<std::endl;
    float angle = cal_angle(scale);
    std::cout<<"angle:"<<angle<<std::endl;

    if(angle<0)
        angle=360+angle;

    angle=(int)(angle/0.5)*0.5;

    std::cout<<"angle:"<<angle<<std::endl;

    for(int i=0;i<res.size();i++){
        for(point p : res[i]){
            if(abs(angle-p.angle)<=1){
                std::cout<<"x: "<<p.x<<" y:"<<p.y<<std::endl;
            }
        }
    }

    int flag=-1;
    for(int i=0;i<res.size();i++){
        for(point p : res[i]){
            if(abs(angle-p.angle)<=1){
                flag=i;
                break;
            }
        }
    }
    std::cout<<"lock:"<<flag<<std::endl;
    //调试
    if(flag==-1){
        for(int i=0;i<res.size();i++){
            int p_=0;
            for(point p : res[i]){
                std::cout<<"i:"<<i<<" p:"<<p_++<<" angle"<<p.angle<<std::endl;
            }
        }
    }

    std::vector<float> vec_new;
    for(point& p : res[flag]){
        vec_new.push_back(sqrt(pow(p.x,2)+pow(p.y,2)));
    }

    float tx=0;
    float ty=0;
    for(int k=0;k<res[flag].size();k++){
        tx+=res[flag][k].x;
        ty+=res[flag][k].y;
    }

    nx = tx/res[flag].size();
    ny = ty/res[flag].size();
    std::cout<<nx<<"m, "<<ny<<"m"<<std::endl;

    cv::Point p2(nx*100+400,ny*100+400);
    cv::circle(img,p2,3,cv::Scalar(255,255,255),-1);

    //初始化kalman
    KF=cv::KalmanFilter(stateNum, measureNum, 0);
    /*初始化状态转移矩阵A 和 测量矩阵H*/
	KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //转移矩阵A
	cv::setIdentity(KF.measurementMatrix);                                             //测量矩阵H

	/*两个噪声的协方差矩阵*/
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));                        //测量噪声方差矩阵R

	/*0时刻的后验状态和协方差*/
	rng.fill(KF.statePost, cv::RNG::UNIFORM, 0, 20000);   //初始状态值x(0)
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));                                  //后验估计协方差矩阵P

	measurement = cv::Mat::zeros(measureNum, 1, CV_32F);           //测量向量z(k)

    measurement.at<float>(0) = nx*1000;
    measurement.at<float>(1) = ny*1000;
    KF.correct(measurement);


    cv::Mat prediction = KF.predict();
    //收敛
    while(!(abs(prediction.at<float>(0)-nx)<0.1 && abs(prediction.at<float>(1)-ny)<0.1)){
        prediction = KF.predict();
        measurement.at<float>(0) = nx;
        measurement.at<float>(1) = ny;
        KF.correct(measurement);
        // std::cout<<"收敛中... "<<prediction.at<float>(0)<<" "<<prediction.at<float>(1)<<std::endl;
    }
    std::cout<<"lidar: kalman filter has converged"<<std::endl;

    //release
    delete temp;
    
    exit(0);
}

cv::Rect LidarTracker::lidarForsee(std::vector<float> lid,cv::Mat& m){

    // std::cout<<std::endl<<"-----------------lidar-------------------"<<std::endl;
    clock_t startTime, endTime;
    startTime = clock();
    //kalman预测值
    cv::Mat prediction = KF.predict();
    endTime = clock();
    // std::cout<<"kalman predict:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;

    startTime = clock();
    //数据处理
    vector<point> dataset;
    int a=0;
    cv::Mat img(220,440,16,cv::Scalar(0,0,0));
    for(float& f : lid){
        point p;
        p.x=sin(((float)a/2)/180*M_PI)*f;
        p.y=cos(((float)a/2)/180*M_PI)*f;
        p.angle=(float)a/2;
        dataset.push_back(p);

        cv::Point p2((p.x+xmax/2)/xmax*440,(p.y+ymax/2)/ymax*220);
        cv::circle(img,p2,1,cv::Scalar(255,255,255),-1);

        a++;
    }
    endTime = clock();
    // std::cout<<"data ana:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;
    
    startTime = clock();
    //聚类(分类)
    vector<vector<point>> res;
    vector<point>* temp=new vector<point>;
    temp->push_back(dataset[0]);
    for(int i=1;i<dataset.size();i++){
        if(pow(dataset[i-1].x-dataset[i].x,2)+pow(dataset[i-1].y-dataset[i].y,2)<=pow(md,2)){
            temp->push_back(dataset[i]);
        }
        else{
            if(temp->size()<=5){
                temp=new vector<point>;
                temp->push_back(dataset[i]);
                continue;
            }
            res.push_back(*temp);
            temp=new vector<point>;
            temp->push_back(dataset[i]);
        }
        if(i==719){
            if(pow(dataset[i].x-dataset[0].x,2)+pow(dataset[i-1].y-dataset[0].y,2)<=pow(md,2)){
                for(int j=0;j<res[0].size();j++){
                    temp->push_back(res[0][j]);
                }
                res.erase(res.begin());
                res.push_back(*temp);
                // std::cout<<5<<std::endl;
            }
            else{
                res.push_back(*temp);
            }
        }
    }
    endTime = clock();
    // std::cout<<"classify:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;

    //测试显示
    std::default_random_engine generator;
    int minColor = 0;
    int maxColor = 255;
    for(vector<point>& ps:res){
        cv::Scalar randomColor(generator() % (maxColor - minColor + 1) + minColor,
                           generator() % (maxColor - minColor + 1) + minColor,
                           generator() % (maxColor - minColor + 1) + minColor);
        for(point p : ps){
            cv::Point p2((p.x+xmax/2)/xmax*440,(p.y+ymax/2)/ymax*220);
            cv::circle(img,p2,1,randomColor,-1);
        }
    }

    startTime = clock();
    vector<point> ruler;
    for(vector<point>& re : res){
        float tx=0;
        float ty=0;
        for(int i=0;i<re.size();i++){
            tx+=re[i].x;
            ty+=re[i].y;
        }
        point p;
        p.x=tx/re.size();
        p.y=ty/re.size();
        ruler.push_back(p);
    }

    for(int i=0;i<ruler.size();i++){
        cv::Point p2((ruler[i].x+xmax/2)/xmax*440,(ruler[i].y+ymax/2)/ymax*220);
        cv::circle(img,p2,3,cv::Scalar(255,0,255),-1);
    }
    endTime = clock();
    // std::cout<<"ruler:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;

    startTime = clock();
    //匹配
    float min_=1000;
    int flag=-1;
    for(int i=0;i<ruler.size();i++){
        if(pow(ruler[i].x-prediction.at<float>(0),2)+pow(ruler[i].y-prediction.at<float>(1),2)<min_){
            min_=pow(ruler[i].x-prediction.at<float>(0),2)+pow(ruler[i].y-prediction.at<float>(1),2);
            flag=i;
        }
    }
    endTime = clock();
    // std::cout<<"select:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;


    startTime = clock();

    std::vector<float> vec_new;
    for(point& p : res[flag]){
        vec_new.push_back(sqrt(pow(p.x,2)+pow(p.y,2)));
    }
    std::vector<float> _vec_new;
    std::vector<float> _vec_old;
    _vec_new=vec_new;
    _vec_old=vec_old;
    //旧补全
    if(vec_new.size()>=vec_old.size()){
        for(int i=0;i<vec_new.size()-vec_old.size();i++){
            _vec_old.push_back(0.01);
        }
    }
    //新补全
    if(vec_old.size()>vec_new.size()){
        for(int i=0;i<vec_old.size()-vec_new.size();i++){
            _vec_new.push_back(0.01);
        }
    }
    conf = cv::EMD(_vec_new,_vec_old, 1);
    conf=max(1-conf,float(0.0));
    vec_old=vec_new;

    //更新
    nx=ruler[flag].x;
    ny=ruler[flag].y;
    measurement.at<float>(0) = nx;
    measurement.at<float>(1) = ny;
    KF.correct(measurement);
    endTime = clock();
    // std::cout<<"update:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;


    startTime = clock();
    //返回位置
    cv::Rect r;
    r.x=cal_angle_f(-nx,ny)*640-10;
    r.width=20;
    r.y=230;
    r.height=20;

    cv::Point p2((nx+xmax/2)/xmax*440,(ny+ymax/2)/ymax*220);
    cv::circle(img,p2,3,cv::Scalar(255,255,255),-1);
    cv::rectangle(img,cv::Rect(220-5,110-8,10,16),cv::Scalar(0,255,0));
    cv::flip(img, m, -1);
    // m=img;
    endTime = clock();
    // std::cout<<"end:"<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;
    // std::cout<<"------------------------------------------"<<std::endl;

    //追踪效果评定
    // float disT=0;//匹配距离
    // if(lidarStandard.size()==0){
    //     conf=1;
    //     lidarStandard = res[flag];
    // }
    // else{
    //     for(int i=0;i<min(lidarStandard.size(),res[flag].size());i++){
    //         disT+=sqrt(pow(lidarStandard[i].x-res[flag][i].x,2)+pow(lidarStandard[i].y-res[flag][i].y,2));
    //     }
    //     disT+=(float)abs((int)lidarStandard.size()-(int)res[flag].size())*maxD;
    //     // std::cout<<"disT"<<disT<<std::endl;
    //     // std::cout<<"maxD"<<(max(lidarStandard.size(),res[flag].size())*maxD)<<std::endl;
    //     conf=1-disT/(max(lidarStandard.size(),res[flag].size())*maxD);
        
    //     lidarStandard = res[flag];
    // }

    //release
    delete temp;
    return r;
}

float LidarTracker::getConf(){ return conf;}

double LidarTracker::cal_angle(double k) {
	if(k > 0.5) {
      return atan((k - 0.5) * PAR_LEN / VER_LEN) / M_PI * 180;
    }
    return 0.0 - (atan((0.5 - k) * PAR_LEN / VER_LEN) / M_PI * 180);
}

double LidarTracker::cal_angle_f(double x,double y) {
    return 0.5+(VER_LEN/y*x)/PAR_LEN;
}

float LidarTracker::squareDistance(point a, point b) {
    float dangle = abs(a.x-b.x)/2;

    float da = a.y;
    float db = b.y;

    return sqrt(da*da+db*db-2*cos(dangle*M_PI/180)*da*db);
}

