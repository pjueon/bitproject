#include "ImageProcessMode.h"

/////////////////////////////////////////////////////////////////////////////
//public
ImageProcessMode::ImageProcessMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "ImageProcess")
{

}

ImageProcessMode::~ImageProcessMode() {}

//======================================================================
mode ImageProcessMode::run() { // 다음 실행할 모드 이름을 리턴
	DebugMsg("run 실행");
	/* 여기에 구체적인 동작을 작성 */
	return mode::Quit;  // 메인 루프 종료
}
//======================================================================

/////////////////////////////////////////////////////////////////////////////
//private