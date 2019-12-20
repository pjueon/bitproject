#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/imgproc/types_c.h>
#include <cmath>
#include <algorithm>

#include "BookImgPreProcessor.h"
#include "UtilityFunctions.h"

using namespace cv;
using namespace std;

BookImgPreProcessor::BookImgPreProcessor()
	: width(0), height(0)
{}


void BookImgPreProcessor::setImg(const cv::Mat& img) {
	width = 1000;
	const double factor = img.cols > width? double(width) / img.cols : 1.0;

	height = static_cast<int>(img.rows * factor);

	resize(img, srcImg, Size(width, height));

	bookAreas.clear();

}

void BookImgPreProcessor::setImg(const std::string& filename) {
	setImg(imread(filename, IMREAD_ANYCOLOR));
	if (srcImg.empty()) throw runtime_error("Image load failed");
}

void BookImgPreProcessor::showResult() {
	if (resultImg.empty()) {
		cerr << "[Error] Please excute run() first" << endl;
		return;
	}

	// ��� ���
	imshow("src", srcImg);
	imshow("result", resultImg);
	imshow("edge", edgeImg);

	waitKey();

}


void BookImgPreProcessor::saveResult(const string& filenamePrefix) {
	if (resultImg.empty()) {
		cerr << "[Error] Please excute run() first" << endl;
		return;
	}

	// ��� ���Ϸ� ����
	string filenameSuffix = ".jpg";
	imwrite(filenamePrefix + "_edge" + filenameSuffix, trimmedEdgeImg);
	imwrite(filenamePrefix + "_fit_result" + filenameSuffix, resultImg);

}

// ª�� ���� ����
void BookImgPreProcessor::removeShortEdges(const int threshold) {

	Mat _output = Mat::zeros(edgeImg.size(), edgeImg.type());

	Mat labels, stats, centroids;
	int nlabels = connectedComponentsWithStats(edgeImg, labels, stats, centroids);

	for (int y = 0; y < height; y++) {
		// at���� ptr�� �����ϴ°� �� ����
		uchar* pointer_input = edgeImg.ptr<uchar>(y);
		uchar* pointer_ouput = _output.ptr<uchar>(y);
		for (int x = 0; x < width; x++) {
			int label = labels.at<int>(y, x);

			int* p = stats.ptr<int>(label);
			// ����� �ȼ����� ���� threshold �����̸� �����ϱ�
			if (label == 0 || p[4] <= threshold) continue;

			pointer_ouput[x] = pointer_input[x];
		}
	}
	_output.copyTo(trimmedEdgeImg);
}


// ������ ���� ��ȯ
double BookImgPreProcessor::getSlope(const Vec4i& line) {
	double x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
	if (x1 == x2) {
		return 10000.0; // ���Ѵ뿡 ���� �ٻ簪
	}
	else {
		return (y2 - y1) / (x2 - x1);
	}

}

// �� ������ ȭ�� ������ ������ ������ Ȯ��
bool BookImgPreProcessor::crossOnScreen(const Vec4i& line1, const Vec4i& line2) {
	double m1 = getSlope(line1), m2 = getSlope(line2);
	double b1 = line1[1] - m1 * line1[0], b2 = line2[1] - m2 * line2[0];

	if (m1 == m2 && b1 == b2) {
		return true;
	}
	else if (m1 == m2) {
		return false;
	}
	else {
		double p_x = (b2 - b1) / (m1 - m2);
		double p_y = m1 * p_x + b1;

		return 0 <= p_x && p_x <= width && 0 <= p_y && p_y <= height;
	}
}

// �� ������ �������� Ȯ��
bool BookImgPreProcessor::isParallel(const Vec4i& line1, const Vec4i& line2) {
	constexpr auto angleThreshold = CV_PI/100.0; 

	auto angle1 = atan2(line1[3] - line1[1], line1[2] - line1[0]);
	auto angle2 = atan2(line2[3] - line2[1], line2[2] - line2[0]);

	auto angleDiff = fitAngleInRange(angle2 - angle1);

	return abs(angleDiff) <= angleThreshold;
}

// �̹��� ���� ���� �������� �ѹ��� �׸���
void BookImgPreProcessor::drawLines(const Mat& inputImg, Mat& outputImg, const vector<Vec4i>& lines, const Scalar color, const int thickness) {
	outputImg = inputImg.clone();
	for (const Vec4i& l : lines)
		line(outputImg, Point(l[0], l[1]), Point(l[2], l[3]), color, thickness, LINE_AA);
}

// �� �� ������ �Ÿ�(L2 norm)
double BookImgPreProcessor::Distance(const Point& A, const Point& B) {
	return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}


// å ������ �ʷϻ����� ��ĥ
void BookImgPreProcessor::fillBookAreas(const Mat& inputImg, Mat& outputImg) {
	Mat mask = Mat::zeros(inputImg.size(), CV_8UC3);
	Scalar Green(0, 255, 0);
	Point points[4];
	const Point* ppt[1] = { points };
	int npt[] = { 4 };
	for (const auto& ba : bookAreas) {
		points[0] = ba[0];
		points[1] = ba[1];
		points[2] = ba[2];
		points[3] = ba[3];
		fillPoly(mask, ppt, npt, 1, Green, LINE_8);
	}

	addWeighted(inputImg, 1.0, mask, 0.3, 0.0, outputImg);
}

// 2���� ���μ��� ���� ����
int BookImgPreProcessor::GapBetweenVLines(const Vec4i& line1, const Vec4i& line2) {
	if (crossOnScreen(line1, line2))
		return 0;
	else
		return min(abs(line2[0] - line1[0]), abs(line2[2] - line1[2]));
}


// å�� ���� �� �ִ� ���� ã��
// �ݵ�� ����(x��ǥ ����)�� ���������� ����� ��!!!
void BookImgPreProcessor::findBookAreas(const vector<Vec4i>& sortedVLines, vector<Vec4i>& boarderLines, const int GapThreshold) {
	size_t totalVLineNum = sortedVLines.size();
	size_t lastBoarderIdx = totalVLineNum;
	for (size_t i = 0; i < totalVLineNum - 1; i++) {

		int gap = GapBetweenVLines(sortedVLines[i], sortedVLines[i + 1]);
		if (!isParallel(sortedVLines[i], sortedVLines[i + 1]) || gap < GapThreshold) continue;

		// å�� �ƴ� ������ �����ϱ�
		{
			// å �����̶�� �����Ǵ� �������� ���������� Ư���ȼ��� �����̸� å�� �ƴѰ����� �Ǵ�
			const auto pixelThreshold = static_cast<int>(0.5 * width); // px

			Mat mask = Mat::zeros(srcImg.size(), CV_8UC1);

			vector<Point> points = { Point{sortedVLines[i][0], sortedVLines[i][1]},
									 Point{sortedVLines[i][2], sortedVLines[i][3]} ,
									 Point{sortedVLines[i + 1][2], sortedVLines[i + 1][3]},
									 Point{sortedVLines[i + 1][0], sortedVLines[i + 1][1]}
									 };
			int npt[] = { 4 };
			const Point* ppt[1] = { &(points[0]) };

			fillPoly(mask, ppt, npt, 1, Scalar(255));

			// ������ ��輱 �κ� ����(0���� ���� �߱�)
			drawLines(mask, mask, { sortedVLines[i] , sortedVLines[i + 1] }, Scalar(0), 5);

			Mat x = edgeImg & mask;

			if (countNonZero(x) < pixelThreshold) continue;
		}
		

		if (lastBoarderIdx != i)boarderLines.push_back(sortedVLines[i]);
		boarderLines.push_back(sortedVLines[i + 1]);
		lastBoarderIdx = i + 1;

		bookAreas.push_back(
			Vec<Point, 4>(
				Point(sortedVLines[i][0], sortedVLines[i][1]),            // Top Left
				Point(sortedVLines[i + 1][0], sortedVLines[i + 1][1]),    // Top Right
				Point(sortedVLines[i + 1][2], sortedVLines[i + 1][3]),    // Bottom Right
				Point(sortedVLines[i][2], sortedVLines[i][3])			  // Bottom Left
				)
		);
	}
}


// å �ѱ� �ѱ��� �̹����� �����ϱ�
void BookImgPreProcessor::saveBookCovers(const string& filenamePrefix) {
	auto imgs = getBookImgs();
	string prefix = "book_";

	for (int i = 0; i < bookAreas.size(); i++) {

		stringstream ss;
		ss << filenamePrefix << "_" << prefix << setw(3) << setfill('0') << i << ".jpg";
		imwrite(ss.str(), imgs[i]);
	}
}

std::vector<cv::Mat> BookImgPreProcessor::getBookImgs() {
	vector<Mat> ret(bookAreas.size());

	for (int i = 0; i < bookAreas.size(); i++) {
		const Vec<Point, 4>& points = bookAreas[i];

		int h = int(max(Distance(points[0], points[3]), Distance(points[1], points[2])));
		int w = int(max(Distance(points[0], points[1]), Distance(points[2], points[3])));

		Vec<Point2f, 4> srcPoints, dstPoints;

		srcPoints[0] = Point2f(points[0]);
		srcPoints[1] = Point2f(points[1]);
		srcPoints[2] = Point2f(points[2]);
		srcPoints[3] = Point2f(points[3]);

		dstPoints[0] = Point2f(0.f, 0.f);
		dstPoints[1] = Point2f(w - 1.f, 0.f);
		dstPoints[2] = Point2f(w - 1.f, h - 1.f);
		dstPoints[3] = Point2f(0.f, h - 1.f);

		Mat transformMat = getPerspectiveTransform(srcPoints, dstPoints);

		//warpPerspective(srcImg, ret[i], transformMat, Size(w, h));
		warpPerspective(grayImg, ret[i], transformMat, Size(w, h));
	}

	return ret;
}


// ������ �� ���μ� ����
void BookImgPreProcessor::GetVLines(const vector<Vec4i>& inputLines, vector<Vec4i>& outputLines, const double verticalThreshold) {
	for (const auto& l : inputLines) {
		if (abs(getSlope(l)) >= verticalThreshold)
		{
			outputLines.push_back(l);
		}
	}
}


// ������ ȭ�� ��ü�� ����
void BookImgPreProcessor::ExtendLineToMax(const Vec4i& inputLine, Vec4i& outputLine) {
	int x1 = inputLine[0], y1 = inputLine[1], x2 = inputLine[2], y2 = inputLine[3];
	double m = getSlope(inputLine);
	double b = y1 - m * x1;

	if (x1 == x2) { // ������
		outputLine[0] = x1;
		outputLine[1] = 0;
		outputLine[2] = x1;
		outputLine[3] = height;
	}
	else if (m == 0) { // ����
		outputLine[0] = 0;
		outputLine[1] = y1;
		outputLine[2] = width;
		outputLine[3] = y1;
	}
	else {
		outputLine[0] = cvRound(-b / m);
		outputLine[1] = 0;
		outputLine[2] = cvRound((height - b) / m);
		outputLine[3] = height;
	}
}


void BookImgPreProcessor::run() {
	// Hough Transform �Ķ����
	constexpr double rho = 1; // distance resolution in pixels of the Hough grid
	constexpr double theta = CV_PI / 180; // angular resolution in radians of the Hough grid
	constexpr int houghThreshold = 100;	 // minimum number of votes(intersections in Hough grid cell)
	constexpr double minLineLength = 150; //minimum number of pixels making up a line
	constexpr double maxLineGap = 40;	//maximum gap in pixels between connectable line segments

	
	if (srcImg.empty()) {
		cerr << "[Error] Please set Image before excute run()" << endl;
		return;
	}



	// ������� ��ȯ
	//Mat gray;
	cvtColor(srcImg, grayImg, COLOR_RGB2GRAY);

	// ���� ����(Canny ���� �����)
	Canny(grayImg, edgeImg, 50, 150);

	//ª�� ���� ����
	removeShortEdges(200);

	// ���� ��ȯ���� ���� ����
	vector<Vec4i> lines;
	HoughLinesP(trimmedEdgeImg, lines, rho, theta, houghThreshold, minLineLength, maxLineGap);

	// ������ �������� ���μ� ����(��밢�� Pi/2 +- Pi/15)
	vector<Vec4i> vLines;
	const double verticalThreshold = tan(CV_PI / 2 - CV_PI / 15);

	GetVLines(lines, vLines, verticalThreshold);

	// ������ ȭ�鿡 ������ ����
	size_t num_of_vertical_lines = vLines.size();
	for (size_t i = 0; i < num_of_vertical_lines; i++) {
		ExtendLineToMax(vLines[i], vLines[i]);
	}

	// x ��ǥ �������� ����(���μ��� ������ ���� ����ϱ� ����)
	auto cmpX = [](const Vec4i& a, const Vec4i& b) {
		if (a[0] != b[0])
			return a[0] < b[0];
		else
			return a[2] < b[2];
	};

	sort(vLines.begin(), vLines.end(), cmpX);

	// ���μ��� ������ ���� ���� �� �����̸� ����
	const int GapThreshold = cvRound(width / 50);
	//getBookImgsvector<Vec<Point, 4>> bookAreas;
	vector<Vec4i> boarderLines;

	findBookAreas(vLines, boarderLines, GapThreshold);

	// å ǥ�� �߶󳻼� �����ϱ�
	//saveBookCovers();

	// ���� ���� ��� �׸���
	resultImg = srcImg.clone();
	Scalar Red(0, 0, 255);

	fillBookAreas(resultImg, resultImg);
	drawLines(resultImg, resultImg, boarderLines, Red, 2);


	//waitKey();
	//destroyAllWindows();

}
