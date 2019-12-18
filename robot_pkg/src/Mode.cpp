#include "Mode.h"
#include "MainMachine.h"
#include "Logger.h"

////////////////////////////////////////////////////////////////////
//public
OperatingMode::OperatingMode(MainMachine* const mainMachine, const string modeName)
	: mainMachine(mainMachine), 
      modeName(modeName), 
	  logger(new Logger(mainMachine, modeName))
{	
	if (this->mainMachine == nullptr) { 
		string ErrorMsg = "Error: modeName[" + modeName + "], mainMachine is nullptr";
		throw logic_error(ErrorMsg);
	}
	if (this->logger == nullptr) { 
		string ErrorMsg = "Error: modeName[" + modeName + "], logger is nullptr";
		throw logic_error(ErrorMsg);
	}
}

OperatingMode::~OperatingMode() {
	delete logger;
}


void OperatingMode::test(){
	logger->DebugMsg("Thi is a test function");
}
