#pragma once
#ifndef MODE_H
#define MODE_H

#include <string>
#include <memory>


class MainMachine;
class Logger;

enum class mode {  
	Standby, AutoDrive, TakePhoto, ImageProcess, Quit  
};
//===============================================================================

class OperatingMode {  
public:
	OperatingMode(MainMachine* const, const std::string);
	virtual ~OperatingMode();
	virtual mode run() = 0;
	virtual void init() = 0;
	virtual void test();  

protected:
	MainMachine* const mainMachine;
	const std::string modeName;

protected:
	const std::unique_ptr<Logger> logger;   
};

#endif

