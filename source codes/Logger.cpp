#include "Logger.h"
#include "MainMachine.h"
#include <iostream>
using namespace std;

Logger::Logger(string modeName, MainMachine* const mainMachine)
	: modeName(modeName), mainMachine(mainMachine)
{}

Logger::~Logger() {}

void Logger::Log(string msg) {
	string debugMsg = "Mode Name: [" + modeName + "], Message: " + msg + "\n";
	cout << debugMsg; //콘솔 출력
	/* 파일에 기록, UI에 출력 etc */
}