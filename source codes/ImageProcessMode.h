#pragma once
#ifndef IMAGEPROCESSMODE_H
#define IMAGEPROCESSMODE_H

#include "Mode.h"

class ImageProcessMode : public OperatingMode {
public:
	explicit ImageProcessMode(MainMachine* const);
	~ImageProcessMode() override;
	mode run() override;

private:  //실제 기능 구현부

};

#endif