#pragma once
#ifndef AUTO_DRIVEMODE_H
#define AUTO_DRIVEMODE_H

#include "Mode.h"

#include <vector>
#include <utility>
#include <memory>

class PathFinder;

class AutoDriveMode : public OperatingMode {
public:
	explicit AutoDriveMode(MainMachine* const);
	~AutoDriveMode() override;
	mode run() override;
	void init() override;
	void test() override;  
  
	void setRoute(const std::vector<std::pair<double, double>>& route);

private:  
	std::vector<std::pair<double, double>> route;
	const std::unique_ptr<PathFinder> pathFinder;

	void printCurrPosition() const;
	bool didArrive(double goalX, double goalY) const;
	double distanceToGoal(double goalX, double goalY) const;
	double angleToGoal(double goalX, double goalY) const;

	void getNextRoute();

	//==Temporary==
	int currRouteIdx;
	std::vector<std::vector<std::pair<double, double>>> destinationList;
	//=============
};

#endif
