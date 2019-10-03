#pragma once
#ifndef AUTODRIVEMODE_H
#define AUTODRIVEMODE_H

#include "Mode.h"

class AutoDriveMode : public OperatingMode {
public:
	explicit AutoDriveMode(MainMachine* const);
	~AutoDriveMode() override;
	mode run() override;

private:  //실제 기능 구현부
	// 1. 카메라에서 사진을 한장 찍어서 멤버로 저장(HardwareControl의 기능 이용)하는 기능
	// 2. 저장된 사진에 정지 마크가 있는지 확인하는 기능
	// 3. 정지 마크에서 현재 위치정보를 읽어내는 기능(QRCODE, OCR etc)
	// 4. 저장된 사진에서 차선 정보(직선) 추출하는 기능
	// 5. 추출한 차선 정보를 가지고 사진 위에 추출한 직선을 그려서 띄워주는 기능
	// 6. 추출한 차선 정보를 가지고 특정방향으로 주행(HardwareControl의 기능 이용)하는 기능
	// etc
};

#endif