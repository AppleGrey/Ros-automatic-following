#pragma once
#include "CTracking.h"


CTracking* c;
std::string lastToken="";
std::string sending="0";//发送缓存

//回调函数(topic 0)
void instruct_out_Callback(const std_msgs::String msg)
{
	std::string ins = msg.data;
    if(ins==lastToken) return;
    std::cout<<ins<<std::endl;
    lastToken = ins;

     //连接命令
    if(ins[0]=='c'){
        std::cout<<"connected"<<std::endl;
        //版本校验
        // std::vector<string> stu;
        // std::stringstream sstr(ins);
        // std::string token;
        // int be=0;
        // while(std::getline(sstr, token, ','))
        // {
        //     stu.push_back(token);
        // }
        // std::cout<<std::atoi(stu[1].c_str())<<" "<<std::atoi(stu[2].c_str())<<" "<<std::atoi(stu[3].c_str())<<" "<<std::atoi(stu[4].c_str())<<std::endl;
        // c->lock(c->selectROI(std::atoi(stu[1].c_str()),std::atoi(stu[2].c_str()),
        //              std::atoi(stu[3].c_str()),std::atoi(stu[4].c_str())));
        sending="c,1";
    }
	//目标锁定
	else if(ins[0]=='1'){
        std::cout<<ins<<std::endl;
        std::vector<string> stu;
        std::stringstream sstr(ins);
        std::string token;
        int be=0;
        while(std::getline(sstr, token, ','))
        {
            stu.push_back(token);
        }
        std::cout<<std::atoi(stu[1].c_str())<<" "<<std::atoi(stu[2].c_str())<<" "<<std::atoi(stu[3].c_str())<<" "<<std::atoi(stu[4].c_str())<<std::endl;
        if(c->lock(c->selectROI(std::atoi(stu[1].c_str()),std::atoi(stu[2].c_str()),
                     std::atoi(stu[3].c_str()),std::atoi(stu[4].c_str()))))
                     sending="1,1";
        else
            sending="-1,0";
	}
    //启动
	else if(ins[0]=='2'){
        sending="2,1";
        std::thread t([&c](){
            c->run();
            sending="3,1";//停车成功
        });
        t.detach();
	}
    //停止
	else if(ins[0]=='3'){
        c->stop();
	}
    //参数配置
	else if(ins[0]=='4'){
        std::cout<<ins<<std::endl;
        std::vector<string> stu;
        std::stringstream sstr(ins);
        std::string token;
        int be=0;
        while(std::getline(sstr, token, ','))
        {
            stu.push_back(token);
        }
        std::cout<<std::atoi(stu[1].c_str())<<" "<<std::atoi(stu[2].c_str())<<" "<<std::atoi(stu[3].c_str())<<" "<<std::atoi(stu[4].c_str())<<std::endl;
        c->setParam(std::atof(stu[1].c_str()),std::atof(stu[2].c_str()),std::atof(stu[3].c_str()),std::atof(stu[6].c_str()));
        sending="4,1";
	}
    //自检
	// else if(ins[0]=='5'){
    //     delete c;
    //     c=new CTracking(0);
	// }
    //启动软件
	else if(ins[0]=='6'){
        // sending="6,0";
        c=new CTracking(0);
        sending="6,1";
	}
    else if(ins[0]=='8'){
        delete c;
	}
    else if(ins[0]=='e'){
        exit(0);
	}
}

int main(int argc, char** argv){
    //启动ros
    ros::init(argc, argv, "our_track2");

    // c=new CTracking(0);

    std::cout << "waitting for connection" << std::endl;
	
    ros::NodeHandle nh_i;
	ros::Subscriber sub = nh_i.subscribe("/topic0",1,instruct_out_Callback);

    // c->lock(c->selectROI());
    // c->run();

    //初始化Topic 0信道
    ros::NodeHandle nh_2;
	ros::Publisher pub=nh_2.advertise<std_msgs::String>("/topic1", 1);
	std::thread t1([&sending,nh_2,pub](){
		while(ros::ok())
		{
			ros::Rate loop_rate(100);
			ros::spinOnce();
            std_msgs::String ss;
            ss.data=sending;
            pub.publish(ss);
	    	loop_rate.sleep();
		}
    });
	t1.detach();
    ros::spin();
    // c->lock(c->selectROI());
    // c->run();
    // ros::Rate loop_rate(20);
    // while(1){
    //     ros::spinOnce();
    //     loop_rate.sleep();;
    // }
}