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

private:
	void SetMode(mode);

private:
	mode currentMode;
	unordered_map<mode, OperatingMode*> Operation;
	// 그 외 모드간 공유해야하는 자원들(ex> 책장 사진, 책 정보, etc)
};
#endif
