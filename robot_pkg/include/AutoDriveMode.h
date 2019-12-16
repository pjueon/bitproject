#pragma once
#ifndef AUTODRIVEMODE_H
#define AUTODRIVEMODE_H

#include "Mode.h"
#include <vector>
#include <utility>

class PathFinder;

class AutoDriveMode : public OperatingMode {
public:
	explicit AutoDriveMode(MainMachine* const);
	~AutoDriveMode() override;
	mode run() override;
	virtual void init() override;  
	void setRoute(const std::vector<std::pair<double, double>>& route);

private:  
	std::vector<std::pair<double, double>> route;
	PathFinder* pathFinder;


	void printCurrPosition() const;
	bool didArrive(double goalX, double goalY) const;
	double distanceToGoal(double goalX, double goalY) const;
	double angleToGoal(double goalX, double goalY) const;

	void getNextRoute();

	//==Temporary==
	int currRouteIdx;
	vector<vector<pair<double, double>>> destinationList;
	//=============
};

#endif
