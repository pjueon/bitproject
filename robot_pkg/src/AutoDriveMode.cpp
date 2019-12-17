#include "AutoDriveMode.h"
#include "Logger.h"
#include "UtilityFunctions.h"
#include "ros/ros.h"
#include "MainMachine.h"

#include "PathFinder.h"

#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

using namespace std;

constexpr double PI = 3.141592;

constexpr bool CLEAR = true;
constexpr bool BLOCKED = false;  

/////////////////////////////////////////////////////////////////////////////
//public
AutoDriveMode::AutoDriveMode(MainMachine* const mainMachine)
	:OperatingMode(mainMachine, "AutoDrive"), currRouteIdx(0), pathFinder(new PathFinder())
{}

AutoDriveMode::~AutoDriveMode() {
	delete pathFinder;
}

//----------------
void AutoDriveMode::init(){
	getNextRoute();
}


//----------------
void AutoDriveMode::test(){
	logger->DebugMsg("This is a test code.");
}  

//======================================================================
mode AutoDriveMode::run() { 
	logger->DebugMsg("run() start!");

	if(route.size() == 0){
		logger->DebugMsg("\n\n===========\n[WARN] Empty route!\n===========\n\n");
		//return mode::Quit;
	}


	for(auto const& goal : route){
		while(ros::ok()){
		   	const double goalX = goal.first;
			const double goalY = goal.second;

		        mainMachine->updatePosition();

			double angle = angleToGoal(goalX, goalY);
			double angleDiff = fitAngleInRange(mainMachine->getYaw() - angle);
		    		
			if (abs(angleDiff) > PI/10.0) {

				if (angleDiff < 0.0) {
					mainMachine->turnLeft();
				}
				else {
					mainMachine->turnRight();
				}
			}
			else {
				if (abs(angleDiff) < PI/36.0) {
					mainMachine->moveForward();
				}
				else if (angleDiff < 0.0) {
					mainMachine->moveFrontLeft();
				}
				else {
					mainMachine->moveFrontRight();
				}
			}
		     
			if (didArrive(goalX, goalY)) {
				//logger->DebugMsg("arrived to the temporary goal!\n");

				mainMachine->stop();
				sleep(300);
				break;
			}
			//logger->DebugMsg("distance to temporary goal(\n", goalX, ", ", goalY, ") =", distanceToGoal(goalX, goalY));

			//printCurrPosition();
					
			sleep(50);
			mainMachine->stop();
			sleep(5);	
		}
	}

//	route.clear();
//	route.shrink_to_fit();	
	
	logger->DebugMsg("run() end!");
	sleep(1000);

	if(mainMachine->isDestBookshelf()) return mode::TakePhoto; 

	bool doesNextExist = mainMachine->nextDestination();
	if(!doesNextExist) return mode::Quit;
	
	return mode::AutoDrive;
}


//======================================================================

void AutoDriveMode::setRoute(const std::vector<std::pair<double, double>>& route){
	this->route = route;
}



/////////////////////////////////////////////////////////////////////////////
//private


void AutoDriveMode::printCurrPosition() const {
	logger->DebugMsg("[current position] x: " , mainMachine->getX() , ", y: " , mainMachine->getY(), ", yaw: ", mainMachine->getYaw());
}


bool AutoDriveMode::didArrive(double goalX, double goalY) const {
	constexpr double epsilon = 0.05;
	return distanceToGoal(goalX, goalY) < epsilon;
}

double AutoDriveMode::distanceToGoal(double goalX, double goalY) const {
	return Distance(mainMachine->getX(), mainMachine->getY(), goalX, goalY);
}

double AutoDriveMode::angleToGoal(double goalX, double goalY) const {
	return atan2(goalY - mainMachine->getY(), goalX - mainMachine->getX());
}


void AutoDriveMode::getNextRoute(){
	logger->DebugMsg("getNextRoute() start!");

	pathFinder->setcompressionRate(5);

	if(!pathFinder->loadMapData()) {
		logger->DebugMsg("failed to load map data!");
		throw runtime_error("");
	}
	
	//debug
	logger->DebugMsg("map data loaded!");

	mainMachine->updatePosition();

	pathFinder->setStartPoint(mainMachine->getX(), mainMachine->getY());
	pathFinder->setGoal(mainMachine->getDestX(), mainMachine->getDestY());

	auto route = pathFinder->getRoute();

	setRoute(route);

	//debug
	for(const auto& p : route){
		cout << "[route(real)] " << p.first << ", " << p.second << endl;
	}

}


