// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <algorithm>
// #include <time.h>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include <vector>

// #include "fdssttracker.hpp"


// // #include <windows.h>
// //#include <dirent.h>



// using namespace std;
// using namespace cv;

// std::vector <cv::Mat> imgVec;

// int main(int argc, char* argv[]){

// 	if (argc > 5) return -1;

// 	bool HOG = true;
// 	bool FIXEDWINDOW = false;
// 	bool MULTISCALE = true;
// 	bool SILENT = true;
// 	bool LAB = false;
// 	// Create KCFTracker object
// 	FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

// 	// DSSTTracker tracker;

// 	int count = 1;
// 	cv::Mat processImg;
// 	char name[7];
// 	std::string imgName;
// 	std::string imgPath = "F:\\fDSST_cpp-master\\build\\vid_test\\1\\";

// 	//get init target box params from information file
// 	std::ifstream initInfoFile;
// 	std::string fileName = imgPath + "initInfo.txt";
// 	initInfoFile.open(fileName);
// 	std::string firstLine;
// 	std::getline(initInfoFile, firstLine);
// 	float initX, initY, initWidth, initHegiht;
// 	char ch;
// 	std::istringstream ss(firstLine);
// 	ss >> initX, ss >> ch;
// 	ss >> initY, ss >> ch;
// 	ss >> initWidth, ss >> ch;
// 	ss >> initHegiht, ss >> ch;

	

// 	double duration = 0;

// 	VideoCapture capture;
// 	capture.open("F:\\fDSST_cpp-master\\build\\vid_test\\33.mp4");
// 	int frames = capture.get(CAP_PROP_FRAME_COUNT);
// 	cv::namedWindow("windows", cv::WINDOW_NORMAL);
// 	cv::resizeWindow("windows", 1280, 720);
// 	for (;;)
// 	{
// 		auto t_start = clock();
		
// 		if (count == frames)break;
// 		cout << "1:" << count << " 2: " << frames << endl;
		
// 		//processImg = cv::imread(imgFinalPath, CV_LOAD_IMAGE_COLOR);
// 		capture >> processImg;

// 		if (processImg.empty())
// 		{
// 			break;
// 		}
// 		cvtColor(processImg, processImg, COLOR_BGR2GRAY);

// 		cv::Rect showRect;
// 		if (count == 1)
// 		{
// 			cv::Rect initRect;
// 			cv::namedWindow("select", cv::WINDOW_NORMAL);
// 			cv::resizeWindow("select", 1280, 720);
// 			initRect = selectROI("select", processImg);
// 			tracker.init(initRect, processImg);
// 			showRect = initRect;
// 		}
// 		else{
// 			showRect = tracker.update(processImg);
// 		}
// 		auto t_end = clock();

// 		cout << "FPS:" << 1000 / (double)(t_end - t_start) << endl;


// 		cv::rectangle(processImg, showRect, cv::Scalar(0, 255, 0));
// 		cv::imshow("windows", processImg);
// 		cv::waitKey(1);
// 		count++;
// 	}
// 	return 0;
// }
