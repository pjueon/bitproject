#include "ImageProcessMode.h"
#include "Logger.h"
#include "UtilityFunctions.h"
#include "MainMachine.h"

#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;


/////////////////////////////////////////////////////////////////////////////
//public
ImageProcessMode::ImageProcessMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "ImageProcess")
{}

//----------------
ImageProcessMode::~ImageProcessMode() {}


//----------------
void ImageProcessMode::init(){
	logger->DebugMsg("init() called");
}

//----------------
void ImageProcessMode::test(){
	logger->DebugMsg("This is a test code.");
}  

//======================================================================
mode ImageProcessMode::run() { 
	logger->DebugMsg("run() start!");
	logger->DebugMsg("Doing Image Processing...(Just Test)");
	sleep(3*1000);

	auto img = mainMachine->getBookshelfImg();

	string filename = "bookshelf" + to_string(mainMachine->getBookshelfID());
	filename += ".jpg";

	string path = "/home/jetbot/catkin_ws/src/bitproject/";
	logger->DebugMsg("file name: ", filename);
	logger->DebugMsg("Path: ", path);

	imwrite(path + filename, img);

	logger->DebugMsg("Done!!(Just Test)");

	bool doesNextExist = mainMachine->nextDestination();
	if(!doesNextExist) return mode::Quit;	
	return mode::AutoDrive;  
}
//======================================================================

