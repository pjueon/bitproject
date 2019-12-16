#include <chrono>
#include <thread>
#include <cmath>

#include "UtilityFunctions.h"

using namespace std;

constexpr double PI = 3.141592;

double Distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double fitAngleInRange(double angle) {

	if (-PI < angle && angle <= PI) {
		return angle;
	}

	double n = floor((PI - angle) / (2 * PI));
	return angle + 2 * n * PI;
}


void sleep(int m) {
	this_thread::sleep_for(chrono::milliseconds(m));
}
