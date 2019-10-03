#pragma once
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
using namespace std;

class MainMachine;

class Logger {
public:
	Logger(string modeName, MainMachine* const mainMachine);
	~Logger();
	void Log(string msg);

private:
	const string modeName;
	MainMachine* const mainMachine;
};
#endif