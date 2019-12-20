#include "ImageProcessMode.h"
#include "Logger.h"
#include "UtilityFunctions.h"
#include "MainMachine.h"
#include "ImageDB.h"
#include "BookImgPreProcessor.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <sstream>

using namespace std;
using namespace cv;


/////////////////////////////////////////////////////////////////////////////
//public
ImageProcessMode::ImageProcessMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "ImageProcess"), db(make_unique<ImageDB>()), pre_processor(make_unique<BookImgPreProcessor>())
{
	constexpr auto filename = "/home/jetbot/catkin_ws/src/bitproject/ImageDB/imgdb.vtree";
	db->load(filename);
}

//----------------
ImageProcessMode::~ImageProcessMode() = default;


//----------------
void ImageProcessMode::init(){
	logger->DebugMsg("init() called");
}

//----------------
void ImageProcessMode::test(){

	logger->DebugMsg("This is a test code.");

	constexpr auto testFilename = "/home/jetbot/catkin_ws/src/bitproject/test_imgs/bookshelfTest.jpg";
	Mat bookshelfImg = imread(testFilename);

	if (bookshelfImg.empty()) {
		cerr << "Image load failed: " << testFilename << endl;
		return ;
	}


	pre_processor->setImg(bookshelfImg);
	pre_processor->run();

	auto bookImgs = pre_processor->getBookImgs();
	vector<string> bookNames;
	bookNames.reserve(bookImgs.size());

	logger->DebugMsg("num of books in the bookshelf photo: ", bookImgs.size());

	// result !!!!
	for(const auto& img: bookImgs){
		bookNames.emplace_back(db->search(img));
	}
	// result !!!!


	// DEBUG
	stringstream ss;
	//ss << "Books in bookshelf " << mainMachine->getBookshelfID() << ": "<< endl;
	ss << "Books in bookshelf " << endl;
	for(const auto& bookName : bookNames){
		ss << bookName << endl;
	}
	//test
	logger->DebugMsg(ss.str());


	//save imgs (for debug)
	logger->DebugMsg("Saving Files for debuging...");
	//string filename = "bookshelf" + to_string(mainMachine->getBookshelfID()) + ".jpg";
	string filename = "bookshelf" + to_string(mainMachine->getBookshelfID()) + ".jpg";


	const string path = "/home/jetbot/catkin_ws/src/bitproject/test_imgs/";
	//logger->DebugMsg("file name: ", filename);
	//logger->DebugMsg("Path: ", path);

	imwrite(path + filename, bookshelfImg);
	pre_processor->saveBookCovers(path + filename);
	pre_processor->saveResult(path + filename);

	pre_processor->showResult();

	logger->DebugMsg("Done!!(Just Test)");

}  

//======================================================================
mode ImageProcessMode::run() { 
	logger->DebugMsg("run() start!");
	logger->DebugMsg("Doing Image Processing...(Just Test)");
	//sleep(3*1000);

	auto bookshelfImg = mainMachine->getBookshelfImg();
	pre_processor->setImg(bookshelfImg);
	pre_processor->run();

	auto bookImgs = pre_processor->getBookImgs();
	vector<string> bookNames;
	bookNames.reserve(bookImgs.size());

	// result !!!!
	for(const auto& img: bookImgs){
		bookNames.emplace_back(db->search(img));
	}
	// result !!!!


	// DEBUG
	stringstream ss;
	ss << "Books in bookshelf " << mainMachine->getBookshelfID() << ": "<< endl;
	for(const auto& bookName : bookNames){
		ss << bookName << endl;
	}
	//test
	auto result = ss.str();

	logger->DebugMsg(result);
	mainMachine->sendBookResult(result);


	//save imgs (for debug)
	logger->DebugMsg("Saving Files for debuging...");
	string filename = "bookshelf" + to_string(mainMachine->getBookshelfID()) + ".jpg";

	const string path = "/home/jetbot/catkin_ws/src/bitproject/test_imgs/";
	//logger->DebugMsg("file name: ", filename);
	//logger->DebugMsg("Path: ", path);

	imwrite(path + filename, bookshelfImg);
	pre_processor->saveBookCovers(path + filename);
	pre_processor->saveResult(path + filename);

	logger->DebugMsg("Done!!(Just Test)");

	bool doesNextExist = mainMachine->nextDestination();
	if(!doesNextExist) return mode::Quit;	
	return mode::AutoDrive;  
}
//======================================================================

