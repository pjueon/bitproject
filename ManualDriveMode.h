#pragma once
#ifndef MANUALDRIVEMODE_H
#define MANUALDRIVEMODE_H

#include "Mode.h"

class ManualDriveMode : public OperatingMode {
public:
	explicit ManualDriveMode(MainMachine* const);
	~ManualDriveMode() override;
	mode run() override;

private:  //실제 기능 구현부
	
};

#endif