#pragma once
#ifndef TAKEPHOTOMODE_H
#define TAKEPHOTOMODE_H

#include "Mode.h"

class TakePhotoMode : public OperatingMode {
public:
	explicit TakePhotoMode(MainMachine* const);
	~TakePhotoMode() override;
	mode run() override;

private:  //실제 기능 구현부

};
#endif