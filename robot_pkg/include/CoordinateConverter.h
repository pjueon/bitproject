#pragma once
#ifndef COORDINATE_CONVERTER_H
#define COORDINATE_CONVERTER_H

#include <utility>

class CoordinateConverter{
private:
	double origin_x;
	double origin_y;
	double resolution;

public:
	CoordinateConverter(double origin_x, double origin_y, double resolution);
	void setOrigin(double origin_x, double origin_y);
	void setOrigin(const std::pair<double, double>& real_xy);
	void setResolution(double resolution);
	void setVariables(double origin_x, double origin_y, double resolution);


	std::pair<int, int> toMapXY(double real_x, double real_y) const;
	std::pair<int, int> toMapXY(const std::pair<double, double>& real_xy) const;

	std::pair<double, double> toRealXY(int map_x, int map_y) const;
	std::pair<double, double> toRealXY(const std::pair<int, int>& map_xy) const;
};

#endif







