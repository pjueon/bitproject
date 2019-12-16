#include "CoordinateConverter.h"
#include <utility>
#include <tuple>

//===========================================================
CoordinateConverter::CoordinateConverter(double origin_x, double origin_y, double resolution):origin_x(origin_x), origin_y(origin_y), resolution(resolution){}

//===========================================================
void CoordinateConverter::setOrigin(const std::pair<double, double>& real_xy){ std::tie(origin_x, origin_y) = real_xy; }

//===========================================================
void CoordinateConverter::setOrigin(double origin_x, double origin_y){ setOrigin({origin_x, origin_y}); }

//===========================================================
void CoordinateConverter::setResolution(double resolution){ this->resolution = resolution; }

//===========================================================
void CoordinateConverter::setVariables(double origin_x, double origin_y, double resolution){
	setOrigin(origin_x, origin_y);
	setResolution(resolution);
}


//===========================================================
std::pair<int, int> CoordinateConverter::toMapXY(double real_x, double real_y) const {
	int map_x = int((real_x - origin_x) / resolution);
	int map_y = int((real_y - origin_y) / resolution);
	return { map_x, map_y };
}

//===========================================================
std::pair<int, int> CoordinateConverter::toMapXY(const std::pair<double, double>& real_xy) const {
	auto [x, y] = real_xy;
	return toMapXY(x, y);
}

//===========================================================
std::pair<double, double> CoordinateConverter::toRealXY(int map_x, int map_y) const {
	double real_x = origin_x + map_x * resolution;
	double real_y = origin_y + map_y * resolution;
	return { real_x, real_y };
}

//===========================================================
std::pair<double, double> CoordinateConverter::toRealXY(const std::pair<int, int>& map_xy) const {
	auto [x, y] = map_xy;
	return toRealXY(x, y);
}

