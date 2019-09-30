#pragma once
#ifndef MAINMACHINE_H
#define MAINMACHINE_H

#include <unordered_map>
#include <sstream>
#include <string>
using namespace std;

enum class mode;
class OperatingMode;

class MainMachine {
public:
	MainMachine();
	~MainMachine();
	void StartMainLoop();
	void SetDebugMsg(string);

private:
	void SetMode(mode);
	void ShowDebugMsg();

private:
	mode currentMode;
	unordered_map<mode, OperatingMode*> Operation;
	stringstream logBuffer;  // 디버깅용 로그
	// 그 외 모드간 공유해야하는 자원들(ex> 책장 사진, 책 정보, etc)
	
};
#endif
