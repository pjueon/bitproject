#pragma once
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
using namespace std;

class MainMachine;

class Logger {
public:
	Logger(MainMachine* const mainMachine, const string moduleName);
	~Logger();


	template<typename... T>
	void DebugMsg(T&&... args) const {
		//C++17, Folding Expression
		cout << "[Debug] From: [" << moduleName << "], Message: ";
		(cout << ... << args) << endl; 

	}

private:
	MainMachine* const mainMachine;
	const string moduleName;
};
#endif
