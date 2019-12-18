#pragma once
#ifndef IMAGE_PROCESS_MODE_H
#define IMAGE_PROCESS_MODE_H

#include <memory>

#include "Mode.h"

class ImageDB;
class BookImgPreProcessor;

class ImageProcessMode : public OperatingMode {
public:
	explicit ImageProcessMode(MainMachine* const);
	~ImageProcessMode() override;
	mode run() override;
	void init() override; 
	void test() override;   

private:  
	std::unique_ptr<ImageDB> db;
	std::unique_ptr<BookImgPreProcessor> pre_processor;
};

#endif
