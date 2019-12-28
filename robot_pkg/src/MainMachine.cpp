#include "MainMachine.h"
#include "StandbyMode.h"
#include "AutoDriveMode.h"

#include "TakePhotoMode.h"
#include "ImageProcessMode.h"

#include "Logger.h"
#include "UtilityFunctions.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <iostream>  
#include <stdexcept>
#include <memory>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>


using namespace std;

constexpr double PI = 3.141592;
constexpr auto logFileName = "/home/jetbot/catkin_ws/src/bitproject/log/log.txt"; 

/////////////////////////////////////////////////////////////////////////////
//public
MainMachine::MainMachine(ros::NodeHandle& n) 
	: currentMode(mode::Standby), 
	  logger(make_unique<Logger>(this, "MainMachine")),
 	  n(n), motorPub(n.advertise<std_msgs::String>("/motor", 1)),
	  cameraTogglePub(n.advertise<std_msgs::Bool>("/camera_toggle", 1)),
	  bookResultPub(n.advertise<std_msgs::String>("/book_cpp", 1)),
	  x(0.0), y(0.0), yaw(0.0), destIdx(0), logFileOut(logFileName)
{
	try {
		Operation.reserve(4);
		Operation.emplace(mode::Standby, make_unique<StandbyMode>(this));
		Operation.emplace(mode::AutoDrive, make_unique<AutoDriveMode>(this));
		Operation.emplace(mode::TakePhoto, make_unique<TakePhotoMode>(this));
		Operation.emplace(mode::ImageProcess, make_unique<ImageProcessMode>(this));
	}
	catch (exception& e) { 
		cout << e.what() << endl;
		throw runtime_error("exception from MainMachine constructor"); 
	}

	constexpr auto filename = "/home/jetbot/catkin_ws/src/bitproject/robot_pkg/route_info/destinations.txt";
	ifstream fin(filename);
	
	if(!fin.is_open()) {
		logger->DebugMsg("failed to open ", filename);
		throw runtime_error("");
	}


	string line;
	while (std::getline(fin, line))
	{
		std::istringstream iss(line);
	    double value = 0.0;
	    vector<double> tmp;
		while(iss >> value){
			tmp.push_back(value);
		}

		if(tmp.size() == 4)
			destinations.emplace_back(tmp[0], tmp[1], tmp[2], tmp[3]);		
		else if(tmp.size() == 2)
			destinations.emplace_back(tmp[0], tmp[1], 0.0, -1);
		else{
			logger->DebugMsg("ERROR!!!");
			throw runtime_error("");
		}		
	}

	//debug
	for(const auto& d : destinations){
		logger->DebugMsg("===destinations===");
		logger->DebugMsg("x,y,yaw,bookshelfID: ", d.x, ", ", d.y, ", ",  d.yaw, ", ",  d.bookshelfID);		
	}
	logger->DebugMsg("==========");

}

MainMachine::~MainMachine() {
	stop();
	// for (auto& ModePairs : Operation) { 
	// 	delete ModePairs.second;
	// }
	//delete logger;
}

//============================================================================
void MainMachine::startMainLoop() {
	while (currentMode != mode::Quit){
		mode nextMode = Operation[currentMode]->run();
		changeMode(nextMode);
	}
}
//============================================================================
//temporary
void MainMachine::test(){
	logger->DebugMsg("===test start===");

	Operation[mode::TakePhoto]->test();
	
	logger->DebugMsg("===test end===");
}
//============================================================================


void MainMachine::moveForward() const { move("F"); }
void MainMachine::moveBackward() const { move("B"); }
void MainMachine::moveFrontRight() const { move("FR"); }
void MainMachine::moveFrontLeft() const { move("FL"); }
void MainMachine::moveBackRight() const { move("BR"); }
void MainMachine::moveBackLeft() const { move("BL"); }
void MainMachine::turnRight() const { move("R"); }
void MainMachine::turnLeft() const{ move("L"); }
void MainMachine::stop() const { move("STOP"); }

double MainMachine::getX() const { return x; }
double MainMachine::getY() const { return y; }
double MainMachine::getYaw() const { return yaw; }

double MainMachine::getDestX() const { return destinations[destIdx].x; }
double MainMachine::getDestY() const { return destinations[destIdx].y; }
double MainMachine::getDestYaw() const { return destinations[destIdx].yaw; }
bool MainMachine::isDestBookshelf() const { return destinations[destIdx].bookshelfID >= 0; }
int MainMachine::getBookshelfID() const {return destinations[destIdx].bookshelfID; }


void MainMachine::sendBookResult(const std::string& data) const {
	logger->DebugMsg("=================\nsendBookResult() called\n=================");
	std_msgs::String msg;	
	msg.data = data;
	bookResultPub.publish(msg);
}


void MainMachine::setBookshelfImg(const cv::Mat& img){	bookshelfImg = img; } // [WARN] shallow copy
cv::Mat MainMachine::getBookshelfImg(){ return bookshelfImg; } // [WARN] shallow copy

	

void MainMachine::cameraOn() const {
	logger->DebugMsg("camera on called!!!");
	std_msgs::Bool msg;
	msg.data = true;
	cameraTogglePub.publish(msg);	
}


void MainMachine::cameraOff() const { 
	std_msgs::Bool msg;
	msg.data = false;
	cameraTogglePub.publish(msg);	
}


void MainMachine::updatePosition(){
    tf::StampedTransform transform;

    while(true){	
        try{
            TFlistener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
            break;
        }catch(exception& e){
            cerr << e.what() << endl;
            sleep(300);
        }
    }
    
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    yaw = tf::getYaw(transform.getRotation());

}


bool MainMachine::nextDestination(){
	if(destIdx+1 < destinations.size()){
		destIdx++;
		return true;
	}
	else{
		logger->DebugMsg("DONE!!");
		return false;
	}

}




/////////////////////////////////////////////////////////////////////////////
//private
void MainMachine::move(const string& cmd) const {
	std_msgs::String msg;
	msg.data = cmd;
	motorPub.publish(msg);
}


void MainMachine::changeMode(mode nextMode){
	logger->DebugMsg("changeMode() called");
	if(nextMode != mode::Quit) Operation[nextMode]->init();
	currentMode = nextMode;
}


