#include "PathFinder.h"
#include "AStar.h"
#include "SimpleMapBuilder.h"
#include "CoordinateConverter.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

#include <utility>
#include <tuple>
#include <vector>
#include <algorithm>

using namespace std;

PathFinder::PathFinder() : astar(new AStar()), mapBuilder(new SimpleMapBuilder()),
compressionRate(4), map_w(0), map_h(0), origin_x(0), origin_y(0), resolution(0), 
simple_origin_x(0), simple_origin_y(0), simple_map_resolution(0), 
realStart_x(0.0), realStart_y(0.0), realGoal_x(0.0), realGoal_y(0.0),
rawMapXYConverter(new CoordinateConverter(0.0, 0.0, 1.0)),
simpleMapXYConverter(new CoordinateConverter(0.0, 0.0, 1.0))
{}

//-----------
PathFinder::~PathFinder(){
	delete astar;
	delete mapBuilder;
	delete rawMapXYConverter;
	delete simpleMapXYConverter;
}


//-----------

bool PathFinder::loadMapData(){
	// 

	cout << "[DEBUG] PathFinder loadMap() start" << endl;
 
	auto mapMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(5.0));
	
		
	//fail to load	
	if(mapMsg == nullptr) return false;
	

	cout << "[DEBUG] got msg!!" << endl;	

	map_w = mapMsg->info.width;
	map_h = mapMsg->info.height;
	resolution = mapMsg->info.resolution;
	origin_x = mapMsg->info.origin.position.x;
	origin_y = mapMsg->info.origin.position.y;

	rawMapXYConverter->setVariables(origin_x, origin_y, resolution);
	

	//cout << "origin: " << origin_x << ", " << origin_y << endl;

	simple_map_resolution = resolution * compressionRate;
	simple_origin_x = origin_x + compressionRate/2.0 * resolution;
	simple_origin_y = origin_y + compressionRate/2.0 * resolution;

	simpleMapXYConverter->setVariables(simple_origin_x, simple_origin_y, simple_map_resolution);
	
	//cout << "simple_origin: " << simple_origin_x << ", " << simple_origin_y << endl;

	mapBuilder->setOriginalMapSize(map_w, map_h);



	mapBuilder->setCompressionRate(compressionRate);

	//cout << "[DEBUG] compressionRate: " << compressionRate << endl;

	mapBuilder->setMapData(mapMsg->data);

	astar->setSimpleMap(mapBuilder->getSimpleMap());

	
	cout << "[DEBUG] PathFinder loadMap() end" << endl;
	return true;
}

//-----------
void PathFinder::setGoal(double x, double y){
	auto [goal_x, goal_y] = toSimpleXY(x, y);
	std::tie(realGoal_x, realGoal_y) = {x, y};
	cout << "[DEBUG] Goal: real (" << x << ", " << y << ") simple (" << goal_x << ", " << goal_y << ")" << endl;
	astar->setGoal(goal_x, goal_y);
}


//-----------
void PathFinder::setStartPoint(double x, double y){
	auto [start_x, start_y] = toSimpleXY(x, y);
	std::tie(realStart_x, realStart_y) = {x, y};
	cout << "[DEBUG] Start Point: real (" << x << ", " << y << ") simple (" << start_x << ", " << start_y << ")" << endl;
	astar->setStartPoint(start_x, start_y);
}



//-----------
std::pair<int, int> PathFinder::toSimpleXY(double x, double y) const{
	return simpleMapXYConverter->toMapXY(x, y);
}

//-----------
std::pair<int, int> PathFinder::toMapXY(double x, double y) const{
	return rawMapXYConverter->toMapXY(x, y);
}

//-----------
std::pair<double, double> PathFinder::toRealXY(int x, int y) const{
	return simpleMapXYConverter->toRealXY(x, y);
}


//-----------
std::vector<std::pair<double, double>> PathFinder::getRoute(){
	auto originRoute = astar->findRoute();
	
	vector<pair<double, double>> route;
	std::transform(originRoute.begin(), originRoute.end(), std::back_inserter(route), [this](auto p){ return toRealXY(p.first, p.second); });
	
	if(auto [x, y] = toMapXY(realGoal_x, realGoal_y); mapBuilder->getPaddedMapValue(x, y) == true) route.push_back({realGoal_x, realGoal_y});
	if(route.size() > 1) route[0] = { realStart_x, realStart_y };

	return route;
}



//-----------
void PathFinder::setcompressionRate(int compressionRate){
	this->compressionRate = compressionRate;
}
