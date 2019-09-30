#include "Logger.h"
#include "MainMachine.h"

Logger::Logger(string modeName, MainMachine* const mainMachine)
	: modeName(modeName), mainMachine(mainMachine)
{}

Logger::~Logger() {}

void Logger::Log(string msg) {
	mainMachine->SetDebugMsg("Mode Name: [" + modeName + "], Message: " + msg);
}