#include "Mode.h"
#include "MainMachine.h"
#include "Logger.h"

#include <string>
#include <memory>

using namespace std;

////////////////////////////////////////////////////////////////////
//public
OperatingMode::OperatingMode(MainMachine* const mainMachine, const string modeName)
	: mainMachine(mainMachine), 
      modeName(modeName), 
	  logger(std::make_unique<Logger>(mainMachine, modeName))
{	
	if (this->mainMachine == nullptr) { 
		string ErrorMsg = "Error: modeName[" + modeName + "], mainMachine is nullptr";
		throw runtime_error(ErrorMsg);
	}
	if (this->logger == nullptr) { 
		string ErrorMsg = "Error: modeName[" + modeName + "], logger is nullptr";
		throw runtime_error(ErrorMsg);
	}
}

OperatingMode::~OperatingMode() = default;


void OperatingMode::test(){
	logger->DebugMsg("Thi is a test function");
}
