#pragma once
#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include <utility>
#include <vector>

class AStar;
class SimpleMapBuilder;
class CoordinateConverter;

class PathFinder{
public:
	PathFinder();
	~PathFinder();
	
	bool loadMapData();
	void setGoal(double x, double y);
	void setStartPoint(double x, double y);
	std::vector<std::pair<double, double>> getRoute();
	void setcompressionRate(int compressionRate);

	

	//test
	//std::vector<std::vector<bool>> simpleMap;
	//std::vector<std::vector<bool>> paddedMap;
	//std::vector<std::pair<int, int>> raw_route;

	//std::pair<int, int> toSimpleXY(double x, double y) const;
	

private:
	AStar* astar;
	SimpleMapBuilder* mapBuilder;

	CoordinateConverter* rawMapXYConverter;
	CoordinateConverter* simpleMapXYConverter;

	int compressionRate;

	//original map meta data
	int map_w;
	int map_h;
	double origin_x;
	double origin_y;
	double resolution;
	
	//simple map meta data
	double simple_origin_x;
	double simple_origin_y;
	double simple_map_resolution;

	double realStart_x;
	double realStart_y;

	double realGoal_x;
	double realGoal_y;
	

	//std::vector<std::vector<bool>> simpleMap;
	//std::vector<std::vector<bool>> paddedMap;
	
	std::pair<int, int> toSimpleXY(double x, double y) const;
	std::pair<double, double> toRealXY(int x, int y) const;
	std::pair<int, int> toMapXY(double x, double y) const;


};


#endif
