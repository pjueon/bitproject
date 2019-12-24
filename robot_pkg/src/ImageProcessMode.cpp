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
#include <fstream>

using namespace std;
using namespace cv;


/////////////////////////////////////////////////////////////////////////////
//public
ImageProcessMode::ImageProcessMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "ImageProcess"), db(make_unique<ImageDB>()), pre_processor(make_unique<BookImgPreProcessor>())
{
	constexpr auto filename = "/home/jetbot/catkin_ws/src/bitproject/ImageDB/imgdb_demo.vtree";
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
	const string path = "/home/jetbot/catkin_ws/src/bitproject/test_imgs/";

	for(int i=0; i < 2; i++){
		string filename = "bookshelf" + to_string(i);
		Mat bookshelfImg = imread(path + filename + ".jpg");

		if (bookshelfImg.empty()) {
			logger->DebugMsg("Image load failed: ", filename);
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
		
		pre_processor->saveBookCovers(path + filename);
		pre_processor->saveResult(path + filename);

		pre_processor->showResult();
	}

	logger->DebugMsg("Done!!(Just Test)");

}  

//======================================================================
mode ImageProcessMode::run() { 
	logger->DebugMsg("run() start!");

	auto bookshelfImg = mainMachine->getBookshelfImg();


	pre_processor->setImg(bookshelfImg);
	pre_processor->run();

	auto bookImgs = pre_processor->getBookImgs();
	logger->DebugMsg("got bookImgs, size: ", bookImgs.size());

	string filename = "bookshelf" + to_string(mainMachine->getBookshelfID());
	const string path = "/home/jetbot/catkin_ws/src/bitproject/test_imgs/";

	pre_processor->saveBookCovers(path + filename);
	pre_processor->saveResult(path + filename);

	logger->DebugMsg("saved preprocessed results");


	//save imgs (for debug)
	logger->DebugMsg("Saving Files for debuging...");

	imwrite(path + filename + ".jpg", bookshelfImg);

	
	logger->DebugMsg("Saving Images Done!");

	vector<string> bookNames;
	bookNames.reserve(bookImgs.size());

	// result !!!!
	for(const auto& img: bookImgs){
		try{
			auto bookName = db->search(img);
			bookNames.emplace_back(std::move(bookName));		
		}
		catch(exception& e){
			logger->DebugMsg("Exception: ", e.what());
		}
	}
	logger->DebugMsg("bookNames.size(): ", bookNames.size());


	// DEBUG
	stringstream ss;
	ss << "<Books in bookshelf " << mainMachine->getBookshelfID() << ">"<< endl;
	for(const auto& bookName : bookNames){
		ss << bookName << endl;
	}

	const string bookInfoPath = "/home/jetbot/catkin_ws/src/bitproject/book_info/";
	filename = "books" + to_string(mainMachine->getBookshelfID()) + ".txt";
	
	ifstream books_fin(bookInfoPath + filename);
	if(books_fin.is_open()){
		vector<string> books;
		string line; 
		while(getline(books_fin, line)){
			books.push_back(line);
		}

		vector<string> wrongBooks;

		int idx = 0;
		for(; idx < books.size(); idx++){
			if(idx >= bookNames.size()) break;
			else if(books[idx] != bookNames[idx]) wrongBooks.push_back(bookNames[idx]);
		}

		ss << endl << endl <<"wrong books: " << wrongBooks.size() << endl;
		
		for(const auto& wrongBook: wrongBooks){
			ss << wrongBook << endl;
		}
		ss << endl;
	}
	else{
		logger->DebugMsg("fail to open: ", bookInfoPath + filename);
	}

	//test
	auto result = ss.str();

	logger->DebugMsg(result);
	mainMachine->sendBookResult(result);

	sleep(3*1000);
	logger->DebugMsg("Done!!");

	bool doesNextExist = mainMachine->nextDestination();
	if(!doesNextExist) return mode::Quit;
	
	return mode::AutoDrive;  
}
//======================================================================

