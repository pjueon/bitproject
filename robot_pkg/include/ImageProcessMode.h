#pragma once
#ifndef IMAGE_PROCESS_MODE_H
#define IMAGE_PROCESS_MODE_H

#include "Mode.h"

class ImageProcessMode : public OperatingMode {
public:
	explicit ImageProcessMode(MainMachine* const);
	~ImageProcessMode() override;
	mode run() override;
	virtual void init() override;  

private:  

};

#endif
