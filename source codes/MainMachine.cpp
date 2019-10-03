#include "MainMachine.h"
#include "AutoDriveMode.h"
#include "ManualDriveMode.h"
#include "TakePhotoMode.h"
#include "ImageProcessMode.h"
#include "Logger.h"

#include <iostream>  // 디버깅용
#include <stdexcept>
using namespace std;

/////////////////////////////////////////////////////////////////////////////
//public
MainMachine::MainMachine() 
	: currentMode(mode::AutoDrive), //초기 모드
	  logger(new Logger(this, "MainMachine"))
{
	try {
		Operation[mode::AutoDrive] = new AutoDriveMode(this);
		Operation[mode::ManualDrive] = new ManualDriveMode(this);
		Operation[mode::TakePhoto] = new TakePhotoMode(this);
		Operation[mode::ImageProcess] = new ImageProcessMode(this);
	}
	catch (exception& e) { // 예외 처리
		cout << e.what() << endl;
		throw false; // 강제 종료 유도
	}
}

MainMachine::~MainMachine() {
	for (auto& ModePairs : Operation) { // Operation에 할당한 메모리 모두 해제
		delete ModePairs.second;
	}
	delete logger;
}

//============================================================================
void MainMachine::StartMainLoop() {
	while (currentMode != mode::Quit){
		mode nextMode = Operation[currentMode]->run();
		currentMode = nextMode;
	}
}
//============================================================================

/////////////////////////////////////////////////////////////////////////////
//private



