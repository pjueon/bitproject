#include "TakePhotoMode.h"
#include "Logger.h"
#include "UtilityFunctions.h"
#include "MainMachine.h"
#include "CoordinateConverter.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/CompressedImage.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

constexpr double PI = CV_PI;


/////////////////////////////////////////////////////////////////////////////
//public
TakePhotoMode::TakePhotoMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "TakePhoto"), initYaw(0.0), XYConverter(new CoordinateConverter(0.0, 0.0, 1.0))
{}

//----------------
TakePhotoMode::~TakePhotoMode() {
	delete XYConverter;
}


//----------------
void TakePhotoMode::init(){
	getNextInitYaw();
}

//----------------
void TakePhotoMode::setInitYaw(double angle){
	initYaw = angle;
};


//======================================================================
mode TakePhotoMode::run() { 
	logger->DebugMsg("run() start!");
	logger->DebugMsg("initYaw:", initYaw);
    
	rotateTo(initYaw);
	mainMachine->updatePosition();
	logger->DebugMsg("After initial angle setting, current yaw: ", mainMachine->getYaw());
    
	sleep(1000);

	double correctYaw = angleTune(1.2, 0.7);
	logger->DebugMsg("correctYaw : ", correctYaw);    
	rotateTo(correctYaw);

	mainMachine->updatePosition();
	logger->DebugMsg("After angleTune, current yaw : ", mainMachine->getYaw());

	sleep(2000);

	auto [new_x, new_y] = positionTune();
	logger->DebugMsg("new position : ", new_x, ", ", new_y, ", current position: ", mainMachine->getX(), ", ", mainMachine->getY());
	moveBackTo(new_x, new_y);

	mainMachine->updatePosition();
	logger->DebugMsg("After positionTune, current x, y: ", mainMachine->getX(), ", ", mainMachine->getY());

	rotateTo(correctYaw);

	correctYaw = angleTune(2.0, 0.5);
	mainMachine->updatePosition();
	logger->DebugMsg("new correctYaw : ", correctYaw);  

	sleep(2000);
	rotateTo(correctYaw);

	mainMachine->updatePosition();
	logger->DebugMsg("final yaw : ", mainMachine->getYaw());
	
	sleep(5000);
	logger->DebugMsg("\n\n============\nTook a Photo!!!!!!\n============\n");
	sleep(5000);

	bool doesNextExist = mainMachine->nextDestination();
	if(!doesNextExist) return mode::Quit;
	
	return mode::AutoDrive;
}
//======================================================================

/////////////////////////////////////////////////////////////////////////////
//private

void TakePhotoMode::rotateTo(double angle){
	logger->DebugMsg("rotateTo() start!");

	constexpr int maxDelay = 40;
	constexpr int minDelay = 20;
    
	while(ros::ok()){
		mainMachine->updatePosition();
		double angleDiff = fitAngleInRange(mainMachine->getYaw() - angle);
        
        //logger->DebugMsg("angleDiff: ", angleDiff);
        
		if(abs(angleDiff) > PI/60.0){
				if (angleDiff < 0.0) {
					//logger->DebugMsg("LEFT!!!");
					mainMachine->turnLeft();
				}
				else {
					//logger->DebugMsg("RIGHT!!!");
					mainMachine->turnRight();
				}
		}
		else{
			//logger->DebugMsg("OK!!!");
			sleep(500);
			break;
		}
        
        auto delay = abs(angleDiff) <= PI? minDelay : maxDelay;

		sleep(delay);
		mainMachine->stop();
		sleep(5);
	}

	logger->DebugMsg("rotateTo() end!");
}

//----------------
void TakePhotoMode::moveBackTo(double x, double y){
	logger->DebugMsg("moveBackTo() start!");
	while(ros::ok()){
		mainMachine->updatePosition();

		auto angleToGoal = atan2(y - mainMachine->getY(), x - mainMachine->getX());
		auto angleDiff = fitAngleInRange(PI + mainMachine->getYaw() - angleToGoal);

		if (abs(angleDiff) > PI/2.0) {// forward
			//logger->DebugMsg("FORWARD!!");
			angleDiff = fitAngleInRange(mainMachine->getYaw() - angleToGoal);

			if (abs(angleDiff) > PI/5.0) {
				if (angleDiff < 0.0) {
					mainMachine->turnLeft();
				}	
				else {
					mainMachine->turnRight();
				}
			}
			else{
				if (abs(angleDiff) < PI/36.0) {
					mainMachine->moveForward();
				}
				else if(angleDiff < 0.0){
					mainMachine->moveFrontLeft();
				}
				else{
					mainMachine->moveFrontRight();	
				}
			}
		}
		else{ // backward
			//logger->DebugMsg("BACKWARD!!");
			if (abs(angleDiff) > PI/5.0) {
				if (angleDiff < 0.0) {
					mainMachine->turnRight();
				}
				else{ 
					mainMachine->turnLeft();
				}
			}
			else{
				if (abs(angleDiff) < PI/36.0){
					mainMachine->moveBackward();
				}
				else if(angleDiff < 0.0){
					mainMachine->moveBackRight();
				}
				else{
					mainMachine->moveBackLeft();
				}
			}
		}

		if(Distance(mainMachine->getX(), mainMachine->getY(), x, y) < 0.05){
				mainMachine->stop();
				sleep(500);
				break;
		}

		sleep(50);
		mainMachine->stop();
		sleep(5);
	}

	logger->DebugMsg("moveBackTo() end!");
}




//----------------
double TakePhotoMode::angleTune(double frontRagne, double sideRange) {
	logger->DebugMsg("angleTune() start!");
	Mat frontSide = getFrontLaserScan(frontRagne, sideRange);

	morphologyEx(frontSide, frontSide, MORPH_CLOSE, Mat());


	vector<Vec4i> lines;
	HoughLinesP(frontSide, lines, 1, PI/180, 10, 5, 10);

	double angleError = 0.0;

	if(lines.empty()){
		logger->DebugMsg("[WARN] couldn't detect any line");
	}
	else{
		Vec4i longgestLine = getLonggestLine(lines);
		angleError = atan(-1 * double(longgestLine[2]-longgestLine[0])/double(longgestLine[3]-longgestLine[1]) );
	}

	mainMachine->updatePosition();
	logger->DebugMsg("angleTune() end!");

	return fitAngleInRange(mainMachine->getYaw() + angleError);
}


//----------------
std::pair<double, double> TakePhotoMode::positionTune(){
	logger->DebugMsg("\n\n============\npositionTune() start!");	


	auto mapMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(5.0));
		
	//fail to load	
	if(mapMsg == nullptr) throw runtime_error("Failed to load map data! from TakePhotoMode::positionTune()");
	
	logger->DebugMsg("Map Data loaded");
	
	XYConverter->setVariables(mapMsg->info.origin.position.x, mapMsg->info.origin.position.y, mapMsg->info.resolution);
	
	vector<uchar> mapData(mapMsg->data.size(), 0);
	std::transform(mapMsg->data.begin(), mapMsg->data.end(), mapData.begin(), [](auto v)->uchar{ return v < 0 || v > 40? 255 : 0; } );
	
	
	Mat map = Mat(mapData).reshape(1, mapMsg->info.height); // no copy, time complexity O(1)
	Mat mask = Mat::zeros(map.size(), CV_8UC1);
	
	mainMachine->updatePosition();
	double current_x = mainMachine->getX();
	double current_y = mainMachine->getY();
	double current_yaw = mainMachine->getYaw();

	constexpr double x_range = 0.7;
	constexpr double y_range = 0.7;

	//need to be fixed later	
	vector<Point> points;

	auto pointMaker = [this](double x, double y, double r, double angle) {
							auto [map_x, map_y] = XYConverter->toMapXY(x + r * cos(angle), y + r * sin(angle));
							return Point(map_x, map_y);
					  };

	points.emplace_back(pointMaker(current_x, current_y, x_range/2, current_yaw + PI/2));
	auto [tmp_x, tmp_y] = XYConverter->toRealXY(points[0].x, points[0].y);

	points.emplace_back(pointMaker(tmp_x, tmp_y, y_range, current_yaw));
	tie(tmp_x, tmp_y) = XYConverter->toRealXY(points[1].x, points[1].y);

	points.emplace_back(pointMaker(tmp_x, tmp_y, x_range, current_yaw - PI/2));
	tie(tmp_x, tmp_y) = XYConverter->toRealXY(points[2].x, points[2].y);

	points.emplace_back(pointMaker(tmp_x, tmp_y, y_range, current_yaw + PI));

	const Point* pts[1] = { &(points[0]) };
	int npts[] = { 4 };
	
	fillPoly(mask, pts, npts, 1, Scalar(255));

	//morphologyEX close
	morphologyEx(map, map, MORPH_CLOSE, Mat());


	Mat maskedMap = mask & map;
	Mat labels, stats, centroids;

	int cnt = connectedComponentsWithStats(maskedMap, labels, stats, centroids);

	if(cnt <= 1) return { current_x, current_y };
	
	auto max_p = stats.ptr<int>(1);

	for(int i = 1; i < cnt; i++){
		auto current_p = stats.ptr<int>(i);
		if(current_p[4] > max_p[4]) max_p = current_p;
	}

	auto [bookshelf_x, bookshelf_y] = XYConverter->toRealXY(max_p[0]+max_p[2]/2, max_p[1]+max_p[3]/2);
	logger->DebugMsg("Bookshelf position : ", bookshelf_x, ", ", bookshelf_y, ", current position: ", current_x, ", ", current_y);
	logger->DebugMsg("Distance to Bookshelf : ", Distance(bookshelf_x, bookshelf_y, current_x, current_y));
	
	logger->DebugMsg("positionTune() end!\n============\n");

	constexpr double R = 0.9;	
	return { bookshelf_x + R * cos(PI + current_yaw), bookshelf_y + R * sin(PI + current_yaw) };
}




//----------------
Vec4i TakePhotoMode::getLonggestLine(const vector<Vec4i>& lines){
	if(lines.empty()) return Vec4i{};

	Vec4i longgestLine = lines[0];
	double maxLength = Distance(longgestLine[0], longgestLine[1], longgestLine[2], longgestLine[3]);

	for(const auto& l : lines){
		auto d = Distance(l[0], l[1], l[2], l[3]);
		if(maxLength < d){
			longgestLine = l;
		}	maxLength = d;
	}

	return longgestLine;
}


//----------------
void TakePhotoMode::getNextInitYaw(){
	logger->DebugMsg("getNextInitYaw() start!");
	setInitYaw(mainMachine->getDestYaw());

	cout << "next initYaw: " << mainMachine->getDestYaw() << endl;
}

//----------------
Mat TakePhotoMode::getFrontLaserScan(double frontRange, double sideRange){
	constexpr double scanMapRes = 0.01; // m / px

	int scanRange_x = int(frontRange/scanMapRes);
	int scanRange_y = int(sideRange/scanMapRes);

	int x_offset = 0;
	int y_offset = scanRange_y/2;



	Mat laserScanMap(Mat::zeros(Size(scanRange_x,scanRange_y), CV_8UC1));

	auto msgPtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(5.0));
	if(msgPtr == nullptr){
		logger->DebugMsg("[error]No LaserScan data");	
	}
	else{
		auto count = static_cast<int>(msgPtr->ranges.size());
			
		for(int i = 0; i < count; i++){
			double angle = msgPtr->angle_min + msgPtr->angle_increment * i;

			if(-PI/2 <= angle && angle <= PI/2){
				int x = msgPtr->ranges[i] * cos(angle) / scanMapRes + x_offset;
				int y = msgPtr->ranges[i] * sin(angle) / scanMapRes + y_offset;
					
				if(0 <= x && x < scanRange_x && 0 <= y && y < scanRange_y){
					uchar* row_ptr = laserScanMap.ptr<uchar>(y);
					row_ptr[x] = uchar(255);
				}
			}
		}
	}

	return laserScanMap;
}

//----------------
cv::Mat TakePhotoMode::getPhoto() {
	mainMachine->cameraOn();
	logger->DebugMsg("On!!");

	auto cameraMsg = ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/camera_topic", ros::Duration(10.0));

	if(cameraMsg == nullptr) throw runtime_error("Failed to get camera image");

	try
	{
		cv::Mat frame = cv::imdecode(cameraMsg->data, 1);
		return frame;
	}
	catch (...)
	{
		throw runtime_error("cannot decode image");
	}		
}

