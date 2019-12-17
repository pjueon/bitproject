#pragma once
#ifndef STANDBYMODE_H
#define STANDBYMODE_H

#include "Mode.h"

class StandbyMode : public OperatingMode {
public:
	explicit StandbyMode(MainMachine* const);
	~StandbyMode() override;
	mode run() override;
	void init() override;
	void test() override;    

private:  

};

#endif
