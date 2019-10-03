#include "AutoDriveMode.h"
#include "Logger.h"

/////////////////////////////////////////////////////////////////////////////
//public
AutoDriveMode::AutoDriveMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "AutoDrive")
{

}

AutoDriveMode::~AutoDriveMode() {}

//======================================================================
mode AutoDriveMode::run() { // 다음 실행할 모드 이름을 리턴
	logger->DebugMsg("run 실행");
	/* 여기에 구체적인 동작을 작성 */ 
	
	// 1. 카메라에서 사진을 한장 찍고 사진 데이터를 멤버로 저장
	//	  - HardwareControl의 기능 이용

	// 2. 저장된 사진에 정지 마크가 있는지 확인
	//	  - 정지마크가 있는 경우, 정지하고 사진촬영할지 무시하고 진행할지 판단
	//		(ex> 정지마크에서 위치정보등을 인식하여 이미 촬영한 곳이면 무시)
	//	  - 정지해야하면 정지하고 PhotoTake모드로 전환(return mode::PhotoTake)
	

	// 3. 저장된 사진에서 차선 정보(직선) 추출
	//	  - OpenCV 이용. 엣지 검출 & 허프만변환 etc

	// 4. 추출한 차선 정보를 가지고 사진 위에 추출한 직선을 그려서 띄워주기
	//   - 디버깅 & 시연용, OpenCV 이용

	// 5. 추출한 차선 정보를 가지고 특정방향으로 주행
	//	 - 방향 판단은 차선의 방향, 중심선의 위치 등을 이용해서 계산
	//	 - 방향 판단이 복잡하다면 방향 판단 함수와 주행 함수를 나눌것
	//	 - 방향이 정해지면 일정시간(ex>0.2초) 그 방향으로 주행
	//	 - 주행은 HardwareControl의 기능 이용
	//   - 특정 키보드가 입력됐는지 확인하고 입력 되었으면 ManualDrive모드로 전환(return mode::ManualDrive)
	//     중간에 특정 키를 입력하면 ManualDrive모드로 전환하기 위한 기능 
	//     OpenCV의 waitkey 함수를 쓰면 구현 가능
	//   - 위에서 ManualDrive로 전환하지 않았다면 다음 모드도 AutoDrive모드(return mode::AutoDrive)

	return mode::ManualDrive;  // 테스트용
}
//======================================================================

/////////////////////////////////////////////////////////////////////////////
//private
	// 1. 카메라에서 사진을 한장 찍어서 멤버로 저장(HardwareControl의 기능 이용)하는 기능
	// 2. 저장된 사진에 정지 마크가 있는지 확인하는 기능
	// 3. 정지 마크에서 현재 위치정보를 읽어내는 기능(QRCODE, OCR etc)
	// 4. 저장된 사진에서 차선 정보(직선) 추출하는 기능
	// 5. 추출한 차선 정보를 가지고 사진 위에 추출한 직선을 그려서 띄워주는 기능
	// 6. 추출한 차선 정보를 가지고 특정방향으로 주행(HardwareControl의 기능 이용)하는 기능
	// etc