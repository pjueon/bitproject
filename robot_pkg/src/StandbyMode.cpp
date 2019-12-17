#include "StandbyMode.h"
#include "Logger.h"
#include "UtilityFunctions.h"
#include "MainMachine.h"

/////////////////////////////////////////////////////////////////////////////
//public
StandbyMode::StandbyMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "Standby")
{}

//----------------
StandbyMode::~StandbyMode() {}


//----------------
void StandbyMode::init(){
	logger->DebugMsg("init() called");
}

//----------------
void StandbyMode::test(){
	logger->DebugMsg("This is a test code.");
}  


//======================================================================
mode StandbyMode::run() { 
	logger->DebugMsg("run() start!");
	logger->DebugMsg("please wait...");
	sleep(5*1000);
	return mode::AutoDrive;  
}
//======================================================================

