#pragma once
#ifndef MODE_H
#define MODE_H

#include <string>

using namespace std;

class MainMachine;
class Logger;

enum class mode {  
	Standby, AutoDrive, TakePhoto, ImageProcess, Quit  
};
//===============================================================================

class OperatingMode {  
public:
	OperatingMode(MainMachine* const, const string);
	virtual ~OperatingMode();
	virtual mode run() = 0;
	virtual void init() = 0;
	virtual void test();  

protected:
	MainMachine* const mainMachine;
	const string modeName;

protected:
	Logger* const logger;  
};

#endif

