#pragma once
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "MainMachine.h"

using namespace std;

class MainMachine;

class Logger {
public:
	Logger(MainMachine* const mainMachine, const string moduleName);
	~Logger();


	template<typename... T>
	void DebugMsg(T&&... args) const {
		stringstream ss;

		//C++17, Folding Expression
		ss << "[Debug] From: [" << moduleName << "], Message: ";
		(ss << ... << args) << endl;
		string msg = ss.str();
		cout << msg; 
		mainMachine->logFileOut << msg;
	}

private:
	MainMachine* const mainMachine;
	const string moduleName;
	


};
#endif
