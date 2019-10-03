#pragma once
#ifndef MAINMACHINE_H
#define MAINMACHINE_H

#include <unordered_map>
#include <sstream>
#include <string>
using namespace std;

enum class mode;
class OperatingMode;
class Logger;

class MainMachine {
public:
	MainMachine();
	~MainMachine();
	void StartMainLoop();

private:
	mode currentMode;
	unordered_map<mode, OperatingMode*> Operation;
	Logger* const logger;  // 디버깅용 로거


	// 그 외 모드간 공유해야하는 자원들(ex> 책장 사진, 책 정보, etc)
};
#endif
