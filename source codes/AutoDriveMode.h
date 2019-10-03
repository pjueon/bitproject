#pragma once
#ifndef AUTODRIVEMODE_H
#define AUTODRIVEMODE_H

#include "Mode.h"

class AutoDriveMode : public OperatingMode {
public:
	explicit AutoDriveMode(MainMachine* const);
	~AutoDriveMode() override;
	mode run() override;

private:  //실제 기능 구현부

};

#endif