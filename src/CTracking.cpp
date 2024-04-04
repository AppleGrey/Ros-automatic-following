#include "CTracking.h"
#include"opencv2/imgproc/imgproc_c.h"
#include <chrono>

std::pair<double, double> speeds=std::make_pair(0,0);//电机操作缓存
sensor_msgs::Image m;//发送图像缓存
cv::Mat* receive=nullptr;//当前缓存的图像
cv::Mat out;//缓存图像2
cv::Mat ml;//缓存雷达输出
int nstate=0;//初始化信号
double lastDis=100;

float s_angle;
float s_distance;
std::string s_mode="STM";
int s_time=0;
float s_cs=0;
float s_ls=0;

double obj_min_angle = 0;
double obj_max_angle = 0;



//回调函数
void vidCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try{
		Cv_Bridge b;
		cv::Mat cv_ptr;
		cv_ptr=*(b.RosToCv(msg).get());
		cv::cvtColor(cv_ptr, cv_ptr, cv::COLOR_RGB2BGR);

		if(nstate==0){
			if(receive==nullptr){
				receive=new cv::Mat(cv_ptr);
			}
			else{
				*receive=cv_ptr;
			}
			out = *receive;
		}
		else{
			if(receive==nullptr){
				receive=new cv::Mat(cv_ptr);
			}
			else{
				*receive=cv_ptr;
			}
		}
	}
	catch(cv::Exception e){
		std::cout<<"error:(opencv)convert failure:"<<e.what()<<std::endl;
	}
	
}

void CTracking::stop(){
	isStart=false;
}

CTracking::CTracking(int cid)
{

	float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
	float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

	//初始化电机
	ros::NodeHandle nh_;
	ros::Publisher pub=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	std::thread t1([&speeds,nh_,pub](){
		while(ros::ok())
		{
			ros::Rate loop_rate(100);
			ros::spinOnce();
	        geometry_msgs::Twist twist;
	        twist.linear.x = speeds.first;
	        twist.linear.z = 0; 
	        twist.angular.x = 0; 
	        twist.angular.y = 0; 
	        twist.angular.z = 0 - speeds.second; 
	       	pub.publish(twist);
	    	loop_rate.sleep();
		}
    });
	t1.detach();

	//初始化图传
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_2 = it.advertise("/camera2/image_raw", 1);
	Cv_Bridge b;
	out=cv::Mat(480,640, 16, cv::Scalar(255,255,255));//全白图
	std::thread t2([nh,pub_2,&b,&out](){
		ros::Rate loop_rate(32);
		while(ros::ok())
		{
			sensor_msgs::Image m;
			cv::Mat tem;
			// cv::resize(out,tem,cv::Size(out.size().width/2,out.size().height/2));
			m = *(b.CvToRos(out).get());
	       	pub_2.publish(m);
	    	loop_rate.sleep();
		}
    });
	t2.detach();

	//初始化雷达图传
	ros::NodeHandle nh_l;
	image_transport::ImageTransport it_l(nh_l);
	image_transport::Publisher pub_3 = it.advertise("/camera3/image_raw", 1);
	Cv_Bridge b_l;
	ml=cv::Mat(230,450, 16, cv::Scalar(255,255,255));//全白图
	std::thread t3([nh_l,pub_3,&b_l,&ml](){
		ros::Rate loop_rate(32);
		while(ros::ok())
		{
			sensor_msgs::Image m;
			cv::Mat tem;
			m = *(b_l.CvToRos(ml).get());
	       	pub_3.publish(m);
	    	loop_rate.sleep();
		}
    });
	t3.detach();

	//初始化信息反馈
	ros::NodeHandle nh_i;
	ros::Publisher pub_4=nh_i.advertise<std_msgs::String>("/topic2", 1);
	std::thread t4([nh_i,pub_4,&s_angle,&s_distance,&s_mode,&s_time](){
		ros::Rate loop_rate(100);
		while(ros::ok())
		{
			ros::Rate loop_rate(100);
			ros::spinOnce();
            std_msgs::String ss;
            ss.data=to_string(s_angle)+","+to_string(s_distance)+","+s_mode+","+to_string(s_time)+","+to_string(s_cs)+","+to_string(s_ls);
            pub_4.publish(ss);
	    	loop_rate.sleep();
		}
    });
	t4.detach();

	//计时器
	std::thread t5([&s_time](){
		while(1){
			if(nstate==1)
				s_time++;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
    });
	t5.detach();

	//初始化摄像机信号接收(ROS)
	nh_r = new ros::NodeHandle();
	sub = nh_r->subscribe("/camera/image_raw",1,vidCallback);
	init();
}

void CTracking::init(){
	std::cout << "loading modules..." << std::endl;

	/*初始化组件*/
	//初始化摄像头
	bool state=true;
	clock_t startTime, endTime;
	startTime = clock();
	while(receive==nullptr){
		ros::Rate loop_rate(100);
		endTime=clock();
		//5s超时
		if((double)(endTime - startTime)/1000>5000){
			state=false;
			break;
		}
	};
	if(state){
		std::cout << "capture:True" << std::endl;
	}
	else{
		std::cout << "capture:False" << std::endl;
		return;
	}
	

	//初始化雷达
	state=true;
	lidar = new fw::Lidar();
	startTime = clock();
	while(firstLid.size()==0){
		ros::Rate loop_rate(100);
		ros::spinOnce();
		firstLid=lidar->getMsg();
		endTime=clock();
		//5s超时
		if((double)(endTime - startTime)/1000>5000){
			state=false;
			break;
		}
	}
	if(state){
		std::cout << "lidar:True" << std::endl;
	}
	else{
		std::cout << "lidar:False" << std::endl;
		return;
	}
	std::cout << std::endl;


	//初始化fDSST追踪器
	std::cout << "initing fDSST...";

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;
	visionTracker=new FDSSTTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	std::cout << " ok" << std::endl;

	//初始化雷达追踪器
	std::cout << "initing ladarTracking...";
	lidarTracker = new LidarTracker();
	std::cout << " ok" << std::endl;

	//初始化kalman滤波器
    KF=cv::KalmanFilter(stateNum, measureNum, 0);
    /*初始化状态转移矩阵A 和 测量矩阵H*/
	KF.transitionMatrix = (cv::Mat_<float>(16, 16) << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
													0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
													0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
													0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
													0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
													0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
													0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
													0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);  //转移矩阵A
	cv::setIdentity(KF.measurementMatrix);                                             //测量矩阵H

	/*两个噪声的协方差矩阵*/
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));                        //测量噪声方差矩阵R

	/*0时刻的后验状态和协方差*/
	rng.fill(KF.statePost, cv::RNG::UNIFORM, 0, 20000);   //初始状态值x(0)
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));                                  //后验估计协方差矩阵P
	measurement = cv::Mat::zeros(measureNum, 1, CV_32F);           //测量向量z(k)

}

cv::Rect CTracking::selectROI()
{
	std::cout << "select ROI"<<std::endl;
	//视觉
	firstImg=*receive;

	//雷达
	firstLid=lidar->getMsg();
	std::cout<<"size:"<<firstImg.size()<<std::endl;
	cv::namedWindow("select", cv::WINDOW_NORMAL);
	cv::resizeWindow("select", 1280, 720);
	// nstate=1;
	return cv::selectROI("select", firstImg);
}

cv::Rect CTracking::selectROI(int l1,int l2,int r1,int r2)
{
	std::cout << "select ROI"<<std::endl;
	//视觉
	firstImg=*receive;

	//雷达
	firstLid=lidar->getMsg();
	std::cout<<"size:"<<firstImg.size()<<std::endl;
	cv::Rect r(l1,l2,r1-l1,r2-l2);
	// nstate=1;
	return r;
}

bool CTracking::lock(cv::Rect rect)
{
	try{
		// std::cout<<0<<endl;
		std::cout<<"locking:["<<rect.x<<","<<rect.y<<","<<rect.width<<","<<rect.height<<"]"<<std::endl;
		//视觉锁定
		cv::cvtColor(firstImg, firstImg, cv::COLOR_BGR2GRAY);
		try{
			visionTracker->init(rect, firstImg);
		}
		catch(cv::Exception e){
			std::cout<<"(fDSST init)"<<std::endl;
			std::cout<<e.what()<<std::endl;
		}
		std::cout<<0<<std::endl;

		//kalman锁定
		measurement.at<float>(0) = rect.x;
		measurement.at<float>(1) = rect.y;
		std::cout<<1<<std::endl;
		measurement.at<float>(2) = rect.x+rect.width;
		measurement.at<float>(3) = rect.y;
		std::cout<<2<<std::endl;
		measurement.at<float>(4) = rect.x+rect.width;
		measurement.at<float>(5) = rect.y+rect.height;
		measurement.at<float>(6) = rect.x;
		measurement.at<float>(7) = rect.y+rect.height;
		std::cout<<3<<std::endl;
		KF.correct(measurement);
		std::cout<<4<<std::endl;
		cv::Mat prediction;
		try{
			prediction=KF.predict();
		}
		catch(cv::Exception e){
			std::cout<<"(kalman opencv)"<<std::endl;
			std::cout<<e.what()<<std::endl;
		}
		std::cout<<1<<std::endl;
		//收敛
		while(!(abs(prediction.at<float>(0)-rect.x)<0.1 && abs(prediction.at<float>(1)-rect.y)<0.1 &&
				abs(prediction.at<float>(2)-rect.x-rect.width)<0.1 && abs(prediction.at<float>(3)-rect.y)<0.1 &&
				abs(prediction.at<float>(4)-rect.x-rect.width)<0.1 && abs(prediction.at<float>(5)-rect.y-rect.height)<0.1 &&
				abs(prediction.at<float>(6)-rect.x)<0.1 && abs(prediction.at<float>(7)-rect.y-rect.height)<0.1)
			)
		{
			try{
				prediction = KF.predict();
				measurement.at<float>(0) = rect.x;
				measurement.at<float>(1) = rect.y;
				measurement.at<float>(2) = rect.x+rect.width;
				measurement.at<float>(3) = rect.y;
				measurement.at<float>(4) = rect.x+rect.width;
				measurement.at<float>(5) = rect.y+rect.height;
				measurement.at<float>(6) = rect.x;
				measurement.at<float>(7) = rect.y+rect.height;
				KF.correct(measurement);
				// std::cout<<"收敛中... "<<prediction.at<float>(0)<<" "<<prediction.at<float>(1)<<std::endl;
			}
			catch(cv::Exception e){
				std::cout<<"(kalman detect opencv)"<<std::endl;
				std::cout<<e.what()<<std::endl;
			}
			
		}
		std::cout<<"lidar: kalman filter has converged"<<std::endl;

		//雷达锁定
		lidarTracker->init(rect,firstLid);
		return true;
	}
	catch(cv::Exception e){
		std::cout<<"(fDSST init)"<<std::endl;
		std::cout<<e.what()<<std::endl;
		return false;
	}
	
}

void putText(cv::Mat m,string s,cv::Rect r,cv::Scalar color){
    cv::Point org;
	org.x=r.x;
	org.y=r.y-20;

    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.8;
    int thickness = 2;
    
    // 将文本写入到图像上
    cv::putText(m,s,org, fontFace, fontScale, color, thickness);
}


void CTracking::run()
{
	isStart=true;
	nstate=1;
	//直方图参数
	int bins = 30;   // 设置直方图的bin数量
    float range[] = {0, 256};   // 设定直方图的值范围
    const float* histRange = {range};
    bool uniform = true;   // 表示直方图的bins大小应该是等分的
    bool accumulate = false;   // 不要求积累结果

	//记录数据
	long long frame=0;
	
	while (isStart) {
		// std::cout<<std::endl<<"-----------------sys-------------------"<<std::endl;

		clock_t startTime, endTime;
		startTime = clock();

		clock_t startTime_, endTime_;
		startTime_ = clock();
		ros::spinOnce();
		endTime_ = clock();
		// std::cout<<"spinonce:"<<std::to_string((double)(endTime_ - startTime_)/1000)+"ms"<<std::endl;

		startTime_ = clock();
		//图像数据
		cv::Mat processImg;
		auto t_start = clock();
		processImg=*receive;
		
		if (processImg.empty())
		{
			break;
		}

		endTime_ = clock();
		// std::cout<<"pic get:"<<std::to_string((double)(endTime_ - startTime_)/1000)+"ms"<<std::endl;

		startTime_ = clock();
		//雷达数据
		std::vector<float> temp = lidar->getMsg();
		endTime_ = clock();
		// std::cout<<"lidar get:"<<std::to_string((double)(endTime_ - startTime_)/1000)+"ms"<<std::endl;


		startTime_ = clock();
		//视觉预测
		cv::Rect r1;
		try{
			r1 = visionForsee(processImg);
		}
		catch(cv::Exception e){
			std::cout<<"(vision Forsee error):"<<e.what()<<std::endl;
		}

		//视觉预测效果评定
		float vscore=0;
		cv::Mat comp;
		try{
			cv::Rect truer1;
			truer1.x=r1.x<0 ? 0 : (r1.x<640? r1.x : 640);
			truer1.y=r1.y<0 ? 0 : (r1.y<480? r1.y : 480);
			truer1.width=r1.x+r1.width<0 ? r1.width+r1.x : (r1.x+r1.width<640? r1.width : 640-r1.x);
			truer1.height=r1.y+r1.height<0 ? r1.height+r1.y : (r1.y+r1.height<480 ? r1.height : 480-r1.y);
			std::cout<<truer1.x<<" "<<truer1.y<<" "<<truer1.width<<" "<<truer1.height<<" "<<std::endl;
			comp=processImg(truer1).clone();
			cv::cvtColor(comp, comp, cv::COLOR_BGR2GRAY);
		}
		catch(cv::Exception e){
			std::cout<<"CV temp:"<<e.what()<<std::endl;
		}
		if(frame==0){
			//第一帧
			std::cout<<r1.x<<" "<<r1.y<<" "<<r1.width<<" "<<r1.height<<std::endl;
			vscore=1;//置信度为100%
			visionStandard=comp;
		}
		else{
			try{
				cv::Mat histogram1;
				cv::calcHist(&visionStandard, 1, nullptr, cv::noArray(), histogram1, 1, &bins, &histRange, uniform, accumulate);

				cv::Mat histogram2;
				cv::calcHist(&comp, 1, nullptr, cv::noArray(), histogram2, 1, &bins, &histRange, uniform, accumulate);

				vscore = cv::compareHist(histogram1, histogram2, cv::HISTCMP_CORREL);//得到置信度
				std::cout<<"vscore:"<<vscore<<std::endl;
				visionStandard=comp;
			}
			catch(cv::Exception e){
				std::cout<<"(HOG error):"<<e.what()<<std::endl;
			}
			
		}

		endTime_ = clock();
		// std::cout<<"visionForsee:"<<std::to_string((double)(endTime_ - startTime_)/1000)+"ms"<<std::endl;

		startTime_ = clock();

		//雷达预测
		cv::Mat t;
		cv::Mat& t1 = t;
		cv::Rect r2 = lidarForsee(temp,t1);
		ml = t1;
		endTime_ = clock();
		// std::cout<<"lidarForsee:"<<std::to_string((double)(endTime_ - startTime_)/1000)+"ms"<<std::endl;

		//雷达效果评定
		float lscore=lidarTracker->getConf();
		std::cout<<"lscore:"<<lscore<<std::endl;

		startTime_ = clock();

		//坐标融合与修正
		r2.y=r1.y+r1.height/2-r2.height/2;
		cv::Rect r;

		s_ls=lscore;
		s_cs=vscore;

		//决策
		if(vscore>=vmin && lscore>=lmin){
			//正常输出
			//视觉与雷达不匹配
			if(r2.x<r1.x || r2.x>r1.x+r1.width){
				//重新匹配雷达信号
				lidarTracker->init(r1,temp);
				r2.x=r1.x+r1.width/2-10;
			}
			//视觉与雷达匹配
			r.x = r2.x+r2.width/2-max(r1.x+r1.width-r2.x-r2.width/2,r2.x+r2.width/2-r1.x)-10;
			r.width=2*max(r1.x+r1.width-r2.x-r2.width/2,r2.x+r2.width/2-r1.x)+20;
			r.y=r1.y-10;
			r.height=r1.height+20;
		}
		else if(vscore>=vmin && lscore<lmin){
			//雷达不可靠
			lidarTracker->init(r1,temp);
			r2.x=r1.x+r1.width/2-10;
			r.x = r2.x+r2.width/2-max(r1.x+r1.width-r2.x-r2.width/2,r2.x+r2.width/2-r1.x)-10;
			r.width=2*max(r1.x+r1.width-r2.x-r2.width/2,r2.x+r2.width/2-r1.x)+20;
			r.y=r1.y-10;
			r.height=r1.height+20;
		}
		else if(vscore<vmin && lscore>=lmin){
			// //修正视觉
			// cv::Rect rl;
			// rl.x=r2.x+r2.width/2-r1.width/2;
			// rl.y=r2.y+r2.height/2-r1.height/2;
			// rl.width=r1.width;
			// rl.height=r1.height;
			// visionTracker->init(rl, processImg);
			//正常输出
			//视觉与雷达不匹配
			if(r2.x<r1.x || r2.x>r1.x+r1.width){
				//重新匹配雷达信号
				lidarTracker->init(r1,temp);
				r2.x=r1.x+r1.width/2-10;
			}
			//视觉与雷达匹配
			r.x = r2.x+r2.width/2-max(r1.x+r1.width-r2.x-r2.width/2,r2.x+r2.width/2-r1.x)-10;
			r.width=2*max(r1.x+r1.width-r2.x-r2.width/2,r2.x+r2.width/2-r1.x)+20;
			r.y=r1.y-10;
			r.height=r1.height+20;
		}
		else{
			//跟丢-直接退
			std::cout<<"error:lost track"<<std::endl;
			break;
		}

		//更新
		cv::Mat prediction = KF.predict();
		measurement.at<float>(0) = r.x;
		measurement.at<float>(1) = r.y;
		measurement.at<float>(2) = r.x+r.width;
		measurement.at<float>(3) = r.y;
		measurement.at<float>(4) = r.x+r.width;
		measurement.at<float>(5) = r.y+r.height;
		measurement.at<float>(6) = r.x;
		measurement.at<float>(7) = r.y+r.height;
        KF.correct(measurement);

		endTime_ = clock();
		// std::cout<<"endtotal:"<<std::to_string((double)(endTime_ - startTime_)/1000)+"ms"<<std::endl;
		// std::cout<<"--------------------------------------------"<<std::endl;
		endTime = clock();
		std::cout<<"Interface Time : "<<std::to_string((double)(endTime - startTime)/1000)+"ms"<<std::endl;
		std::cout<<"--------------------------------------------"<<std::endl<<std::endl;

		//渲染
		cv::Scalar color(255, 255, 255);

		cv::rectangle(processImg, r1, cv::Scalar(0, 255, 0));
		putText(processImg,"Vision",r1,color);

		cv::rectangle(processImg, r2, cv::Scalar(0, 255, 0));
		putText(processImg,"Lidar",r2,color);


		//kalman预测
		cv::Rect kresult;
		kresult.x=prediction.at<float>(0);
		kresult.y=prediction.at<float>(1);
		kresult.width=prediction.at<float>(2)-prediction.at<float>(0);
		kresult.height=prediction.at<float>(5)-prediction.at<float>(1);
		cv::rectangle(processImg, kresult, cv::Scalar(0, 0, 255));
		putText(processImg,"KF",kresult,color);

		//距离(Lidar)
		s_angle=cal_angle(((double)kresult.x+kresult.width/2)/640);
		obj_min_angle = cal_angle((double)kresult.x/640);
		obj_max_angle = cal_angle((double)(kresult.x+kresult.width)/640);
		double dis = cal_distance(obj_min_angle, obj_max_angle,temp);
		if(isnan(dis)){
			dis = lastDis;
		}
		lastDis=dis;
		s_distance=dis;

		//距离显示
		cv::rectangle(processImg, r, cv::Scalar(0, 255, 255));
		std::string st1 = std::to_string(dis);
		st1 = st1.substr(0, st1.find(".") + 3);
		putText(processImg,"Distance:"+st1+"m",r,color);

		//推理时间
		cv::Rect rt;
		rt.x=0;
		rt.y=100;
		std::string st2 = std::to_string((double)(endTime - startTime)/1000);
		st2 = st2.substr(0, st2.find(".") + 3);
		putText(processImg,"Interface Time : "+st2+"ms",rt,color);
		
		//路径规划
		cv::Rect rr = r;
		//电机映射
		// speeds = cal_speed((double)rr.x/processImg.size().width,(double)rr.width/processImg.size().width,dis);
		speeds = cal_vel(dis,s_angle);

		//避障(未来)
		//..

		out = processImg;
		// cv::imshow("imgCallback",out);
        // cv::waitKey(1);

		//防止内存泄露
		processImg.release();
		t.release();

		frame++;
	}
	nstate=0;
	speeds=std::make_pair(0,0);//速度重置
	s_time=0;
}

pair<double, double> CTracking::cal_vel(double distance, double angle) {
	angle = angle * M_PI / 180;
	if(distance < 0.6) { return {0.0, 0.0}; }
    //定义当前状态
    VectorXd state(5);
    state<< pos_x, pos_y, yaw, speeds.first, speeds.second;//[x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]

    //计算目标点坐标
    Vector2d goal(pos_x + distance * cos(yaw + angle), pos_y + distance * sin(yaw + angle));

	//计算障碍物位置
    vector<Vector2d> obstacles;
	std::vector<float> lid = lidar->getMsg();
	int edge = 1;
	for(int i = -90; i <= 90; i+=3) {
		if(i < obj_min_angle - edge || i > obj_max_angle + edge) {
			int index = ((360 - i) % 360) * 2;
			if(lid[index] > 0.01 && lid[index] < 1.8) {
				Vector2d obstacle(lid[index] * cos(i), lid[index] * sin(i));
				obstacles.push_back(obstacle);
			}
		}
	}

    //初始化dwa
	vector<Vector2d> empty;
    pair<vector<double>, vector<VectorXd>> res = dwa.dwaControl(state,goal,obstacles);
    // state = dwa.kinematicModel(state,res.first,dt);
    return {res.first[0], res.first[1]};
}

cv::Rect CTracking::visionForsee(cv::Mat input)
{	
	cv::cvtColor(input, input, cv::COLOR_BGR2GRAY);
	cv::Rect showRect;
	try{
		showRect = visionTracker->update(input);
	}
	catch(cv::Exception e){
		std::cout<<24<<std::endl;
		std::cout<<e.what()<<std::endl;
	}
	
	return showRect;
}

cv::Rect CTracking::lidarForsee(std::vector<float> input,cv::Mat& t1)
{
	cv::Rect showRect;
	showRect = lidarTracker->lidarForsee(input,t1);
	return showRect;
}

/** 
	* @brief calculate linear and rotation speed
	* @param rect  position of target
	*
	* @return the first value is linear speed and the second is rotation speed
	*/
std::pair<double, double> CTracking::cal_speed(double x,double w,double distance) {


	//calculate the ranges of target angle
	int min_angle = round(cal_angle(x));
	int max_angle = round(cal_angle(x + w));

	//calculate linear speed
	double linear_speed = 0;
    if(distance > Min_distance) {
    	linear_speed = distance * k_linear_speed + h_linear_speed;
	}
    else {
    	linear_speed = 0;
	}

    if(linear_speed > Max_linear_speed){
    	linear_speed = Max_linear_speed;
	}

	//calulate rotation speed with pure pursuit
	double rotation_speed = 0;
    double delta = atan2(2*L*sin((max_angle + min_angle) * M_PI / 360), distance);
    rotation_speed = static_cast<float>(delta);
    if(rotation_speed > Max_rotation_speed) {
      rotation_speed = Max_rotation_speed;
    }
    if(rotation_speed < -Max_rotation_speed) {
      rotation_speed = -Max_rotation_speed;
    }

	// if(isnan(distance)) linear_speed=0;//紧急制动

	return std::make_pair(linear_speed, rotation_speed);
}

double CTracking::cal_angle(double k) {
	if(k > 0.5) {
      return atan((k - 0.5) * PAR_LEN / VER_LEN) / M_PI * 180;
    }
    return 0.0 - (atan((0.5 - k) * PAR_LEN / VER_LEN) / M_PI * 180);
}

double CTracking::cal_distance(double min_angle, double max_angle, const vector<float>& lid) {

	int min_a = round(min_angle);
    int max_a = round(max_angle);

	// auto distances = lid;
	//store all distance in range
	std::vector<double> angles;
    for(int i = min_a; i < max_a; i++) {
      int index = ((360 - i) % 360) * 2;
      if(lid[index] > MIN_DIS && lid[index] < MAX_DIS) {
        angles.push_back(lid[index]);
      }
    //   std::cout << distances[((360 - i) % 360) * 2] << ", ";
    }
    // std::cout << std::endl;

    //remove error values
    if(angles.size() > 4) {
      grubbsFilter(angles);
    }

    //calculate average distance
    float distance = 0;
    int ll = angles.size();
    for(int i = 0; i < angles.size(); i++)
    {
    //   std::cout << angles[i] << ", " ;
      if(angles[i] > MIN_DIS && angles[i] < MAX_DIS) {
        distance += angles[i];
      } else {
        ll--;
      }
    }
    // std::cout << std::endl;
    distance /= ll;
    // std::cout << distance << std::endl;
	return distance;
}