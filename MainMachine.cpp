#include "MainMachine.h"
#include "AutoDriveMode.h"
#include "ManualDriveMode.h"
#include "TakePhotoMode.h"
#include "ImageProcessMode.h"

#include <iostream>  // 디버깅용
using namespace std;

/////////////////////////////////////////////////////////////////////////////
//public
MainMachine::MainMachine() 
	: currentMode(mode::AutoDrive) //초기 모드
{
	Operation[mode::AutoDrive] = new AutoDriveMode(this);
	Operation[mode::ManualDrive] = new ManualDriveMode(this);
	Operation[mode::TakePhoto] = new TakePhotoMode(this);
	Operation[mode::ImageProcess] = new ImageProcessMode(this);
}

MainMachine::~MainMachine() {
	for (auto& ModePairs : Operation) { // Operation에 할당한 메모리 모두 해제
		delete ModePairs.second;
	}
}

void MainMachine::SetDebugMsg(string msg) { logBuffer << msg; }

//============================================================================
void MainMachine::StartMainLoop() {
	while (currentMode != mode::Quit){
		mode nextMode = Operation[currentMode]->run();
		ShowDebugMsg();
		SetMode(nextMode);
	}
}
//============================================================================

/////////////////////////////////////////////////////////////////////////////
//private
void MainMachine::SetMode(mode nextMode) { currentMode = nextMode; }

void MainMachine::ShowDebugMsg() {
	cout << logBuffer.str();
	logBuffer.str("");  // 버퍼 초기화
}

