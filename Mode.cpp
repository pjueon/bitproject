#include "Mode.h"
#include "MainMachine.h"
#include "Logger.h"

OperatingMode::OperatingMode(MainMachine* const mainMachine, const string modeName)
	: mainMachine(mainMachine), 
      modeName(modeName), 
	  logger(new Logger(modeName, mainMachine))
{	
	if (this->mainMachine == nullptr) { 
		string ErrorMsg = "Error: modeName[" + modeName + "], mainMachine이 nullptr 입니다.";
		throw logic_error(ErrorMsg);
	}
	if (this->logger == nullptr) { 
		string ErrorMsg = "Error: modeName[" + modeName + "], logger가 nullptr 입니다.";
		throw logic_error(ErrorMsg);
	}
}

void OperatingMode::DebugMsg(string msg) { logger->Log(msg); }

OperatingMode::~OperatingMode() {
	delete logger;
}