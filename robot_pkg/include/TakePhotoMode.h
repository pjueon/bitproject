#pragma once
#ifndef TAKE_PHOTOMODE_H
#define TAKE_PHOTOMODE_H

#include "Mode.h"

#include <vector>
#include <array>
#include <utility>
#include <memory>


#include <opencv2/opencv.hpp>


#include <ros/ros.h>


class CoordinateConverter;


class TakePhotoMode : public OperatingMode {
public:
	explicit TakePhotoMode(MainMachine* const);
	~TakePhotoMode() override;
	mode run() override;
	void init() override;
	void test() override;  

	void setInitYaw(double);

private: 
	double initYaw;
	const std::unique_ptr<CoordinateConverter> XYConverter;
	cv::Mat getPhoto();
	std::array<double, 4> getYOLOBoxInfo();
	

	void rotateTo(double angle);
	void moveBackTo(double x, double y);
	

	cv::Mat getFrontLaserScan(double frontRange, double sideRange);
	cv::Vec4i getLonggestLine(const std::vector<cv::Vec4i>& lines);
	
	void getNextInitYaw();

	double angleTune(double frontRange, double sideRange);
	std::pair<double, double> getBookshelfPos();

	

};
#endif
