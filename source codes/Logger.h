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

	// DB에 기록하기(DBTool의 기능을 사용)
	// 파일에 기록(로그 메시지, 사진, 동영상, etc)
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