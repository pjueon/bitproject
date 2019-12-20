#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>

#include "UtilityFunctions.h"

using namespace std;
using namespace cv;

constexpr double PI = CV_PI;

//----------------------------------------------------------
double Distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//----------------------------------------------------------
double fitAngleInRange(double angle) {

	if (-PI < angle && angle <= PI) {
		return angle;
	}

	double n = floor((PI - angle) / (2 * PI));
	return angle + 2 * n * PI;
}

//----------------------------------------------------------
void sleep(int m) {
	this_thread::sleep_for(chrono::milliseconds(m));
}


//----------------------------------------------------------
double L1Norm(const vector<float>& input) {
	return  std::accumulate(input.begin(), input.end(), 0.f, [](float a, float b) {return abs(a) + abs(b); });
}

//----------------------------------------------------------
double L1Norm(const vector<double>& input) {
	return std::accumulate(input.begin(), input.end(), 0.0, [](double a, double b) {return abs(a) + abs(b); });
}

//----------------------------------------------------------
double L2Norm(const vector<float>& input) {
	return sqrt(inner_product(input.begin(), input.end(), input.begin(), 0.0));
}

//----------------------------------------------------------
void MatTo1DVector(const Mat& src, vector<uchar>& output) {
	if (src.isContinuous()) {
		output.assign((uchar*)src.data, (uchar*)src.data + src.total());
	}
	else {
		for (int i = 0; i < src.rows; ++i) {
			output.insert(output.end(), src.ptr<uchar>(i), src.ptr<uchar>(i) + src.cols);
		}
	}
}

//----------------------------------------------------------
void MatTo2DVector(const Mat& src, vector<vector<unsigned char>>& output) {
	output.resize(src.rows);
	for (int i = 0; i < src.rows; i++) {
		MatTo1DVector(src.row(i), output[i]);
	}
}

//----------------------------------------------------------
void resizeIfNecessary(const Mat& input, Mat& output) {
	constexpr int minimumLength = 150; // px
	constexpr int maximumLength = 1500; // px

	const int img_width = input.cols;
	const int img_height = input.rows;

	int new_width = img_width;
	int new_height = img_height;

	double resizingFactor = 1.0;

	if (img_height > maximumLength || img_width > maximumLength) {
		resizingFactor = img_width >= img_height ? maximumLength / static_cast<double>(img_width) : maximumLength / static_cast<double>(img_height);

		if (resizingFactor * img_width < minimumLength || resizingFactor * img_height < minimumLength) {
			resizingFactor = img_width <= img_height ? minimumLength / static_cast<double>(img_width) : minimumLength / static_cast<double>(img_height);
		}
	}
	else if (img_width < minimumLength || img_height < minimumLength) {
		resizingFactor = img_width <= img_height ? minimumLength / static_cast<double>(img_width) : minimumLength / static_cast<double>(img_height);
	}
	else {
		return;
	}
	new_width = static_cast<int> (img_width * resizingFactor);
	new_height = static_cast<int> (img_height * resizingFactor);

	resize(input, output, Size(new_width, new_height));
}