#include "Mode.h"
#include "MainMachine.h"
#include "Logger.h"

OperatingMode::OperatingMode(MainMachine* const mainMachine, const string modeName)
	: mainMachine(mainMachine), 
      modeName(modeName), 
	  logger(new Logger(modeName, mainMachine))
{	
	if (this->mainMachine == nullptr) throw logic_error("Error: mainMachine이 nullptr 입니다.");
	if (this->logger == nullptr) throw logic_error("Error: logger가 nullptr 입니다.");
}

void OperatingMode::DebugMsg(string msg) { logger->Log(msg); }

OperatingMode::~OperatingMode() {
	delete logger;
}