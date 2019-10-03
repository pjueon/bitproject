#include "ManualDriveMode.h"
#include "Logger.h"

/////////////////////////////////////////////////////////////////////////////
//public
ManualDriveMode::ManualDriveMode(MainMachine* const mainMachine) 
	:OperatingMode(mainMachine, "ManualDrive")
{

}

ManualDriveMode::~ManualDriveMode() {}

//======================================================================
mode ManualDriveMode::run() { // 다음 실행할 모드 이름을 리턴
	logger->DebugMsg("run 실행");
	/* 여기에 구체적인 동작을 작성 */
	return mode::TakePhoto; 
}
//======================================================================

/////////////////////////////////////////////////////////////////////////////
//private