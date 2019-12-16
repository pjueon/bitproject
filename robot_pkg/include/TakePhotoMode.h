#pragma once
#ifndef TAKEPHOTOMODE_H
#define TAKEPHOTOMODE_H

#include "Mode.h"

#include <vector>
#include <utility>

#include <opencv2/opencv.hpp>


#include <ros/ros.h>


class CoordinateConverter;


class TakePhotoMode : public OperatingMode {
public:
	explicit TakePhotoMode(MainMachine* const);
	~TakePhotoMode() override;
	mode run() override;
	virtual void init() override;  

	void setInitYaw(double);

private: 
	double initYaw;
	CoordinateConverter* XYConverter;
	cv::Mat getPhoto();
	

	void rotateTo(double angle);
	void moveBackTo(double x, double y);
	

	cv::Mat getFrontLaserScan(double frontRange, double sideRange);
	cv::Vec4i getLonggestLine(const std::vector<cv::Vec4i>& lines);
	
	void getNextInitYaw();

	double angleTune(double frontRange, double sideRange);
	std::pair<double, double> positionTune();

	

};
#endif
