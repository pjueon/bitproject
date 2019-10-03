#include "Logger.h"
#include "MainMachine.h"
#include <iostream>
using namespace std;

Logger::Logger(string moduleName, MainMachine* const mainMachine)
	: moduleName(moduleName), mainMachine(mainMachine)
{}

Logger::~Logger() {}

void Logger::Log(string msg) {
#ifndef NDEBUG
	string debugMsg = "[Debug] From: [" + moduleName + "], Message: " + msg + "\n";
	cout << debugMsg; //콘솔 출력
	/* 파일에 기록, UI에 출력 etc */
#else
	// Do Nothing on Release Build
#endif
}

