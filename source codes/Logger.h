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

	// DB에 기록하기(DBComunication을 사용)
	// 파일에 기록
	// UI에 띄울 메시지 작성 
	// etc

	template<typename... T>
	void DebugMsg(T&&... args) const {
#ifndef NDEBUG
		cout << "[Debug] From: [" << moduleName << "], Message: ";
		(cout << ... << args) << endl; //C++17 기능, Folding Expression
#else
		// Do Nothing on Release Build!
#endif
	}

private:
	MainMachine* const mainMachine;
	const string moduleName;
};
#endif