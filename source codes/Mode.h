#pragma once
#ifndef MODE_H
#define MODE_H

#include <string>
using namespace std;

class MainMachine;
class Logger;

enum class mode {  // 모드 이름
	AutoDrive, ManualDrive, TakePhoto, ImageProcess, Quit  // 필요에 따라 추가/삭제
};
//===============================================================================

class OperatingMode {  // 추상클래스
public:
	OperatingMode(MainMachine* const, const string);
	virtual ~OperatingMode();
	virtual mode run() = 0;  // 재정의 필수, 다음으로 수행할 모드 이름을 반환

protected:
	MainMachine* const mainMachine;
	const string modeName;

protected:
	void DebugMsg(string);

private:
	Logger* const logger;  // 디버깅용 로거
};

#endif

