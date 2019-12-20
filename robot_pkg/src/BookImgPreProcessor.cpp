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

	// 결과 출력
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

	// 결과 파일로 저장
	string filenameSuffix = ".jpg";
	imwrite(filenamePrefix + "_edge" + filenameSuffix, trimmedEdgeImg);
	imwrite(filenamePrefix + "_fit_result" + filenameSuffix, resultImg);

}

// 짧은 엣지 제거
void BookImgPreProcessor::removeShortEdges(const int threshold) {

	Mat _output = Mat::zeros(edgeImg.size(), edgeImg.type());

	Mat labels, stats, centroids;
	int nlabels = connectedComponentsWithStats(edgeImg, labels, stats, centroids);

	for (int y = 0; y < height; y++) {
		// at보다 ptr로 접근하는게 더 빠름
		uchar* pointer_input = edgeImg.ptr<uchar>(y);
		uchar* pointer_ouput = _output.ptr<uchar>(y);
		for (int x = 0; x < width; x++) {
			int label = labels.at<int>(y, x);

			int* p = stats.ptr<int>(label);
			// 연결된 픽셀들의 수가 threshold 이하이면 무시하기
			if (label == 0 || p[4] <= threshold) continue;

			pointer_ouput[x] = pointer_input[x];
		}
	}
	_output.copyTo(trimmedEdgeImg);
}


// 직선의 기울기 반환
double BookImgPreProcessor::getSlope(const Vec4i& line) {
	double x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
	if (x1 == x2) {
		return 10000.0; // 무한대에 대한 근사값
	}
	else {
		return (y2 - y1) / (x2 - x1);
	}

}

// 두 직선이 화면 내에서 교점을 갖는지 확인
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

// 두 직선이 평행한지 확인
bool BookImgPreProcessor::isParallel(const Vec4i& line1, const Vec4i& line2) {
	constexpr auto angleThreshold = CV_PI/100.0; 

	auto angle1 = atan2(line1[3] - line1[1], line1[2] - line1[0]);
	auto angle2 = atan2(line2[3] - line2[1], line2[2] - line2[0]);

	auto angleDiff = fitAngleInRange(angle2 - angle1);

	return abs(angleDiff) <= angleThreshold;
}

// 이미지 위에 직선 여러개를 한번에 그리기
void BookImgPreProcessor::drawLines(const Mat& inputImg, Mat& outputImg, const vector<Vec4i>& lines, const Scalar color, const int thickness) {
	outputImg = inputImg.clone();
	for (const Vec4i& l : lines)
		line(outputImg, Point(l[0], l[1]), Point(l[2], l[3]), color, thickness, LINE_AA);
}

// 두 점 사이의 거리(L2 norm)
double BookImgPreProcessor::Distance(const Point& A, const Point& B) {
	return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}


// 책 영역을 초록색으로 색칠
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

// 2개의 세로선들 사이 간격
int BookImgPreProcessor::GapBetweenVLines(const Vec4i& line1, const Vec4i& line2) {
	if (crossOnScreen(line1, line2))
		return 0;
	else
		return min(abs(line2[0] - line1[0]), abs(line2[2] - line1[2]));
}


// 책이 있을 수 있는 영역 찾기
// 반드시 정렬(x좌표 기준)된 수직선들을 사용할 것!!!
void BookImgPreProcessor::findBookAreas(const vector<Vec4i>& sortedVLines, vector<Vec4i>& boarderLines, const int GapThreshold) {
	size_t totalVLineNum = sortedVLines.size();
	size_t lastBoarderIdx = totalVLineNum;
	for (size_t i = 0; i < totalVLineNum - 1; i++) {

		int gap = GapBetweenVLines(sortedVLines[i], sortedVLines[i + 1]);
		if (!isParallel(sortedVLines[i], sortedVLines[i + 1]) || gap < GapThreshold) continue;

		// 책이 아닌 영역을 제거하기
		{
			// 책 영역이라고 추측되는 영역에서 엣지성분이 특정픽셀수 이하이면 책이 아닌것으로 판단
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

			// 영역의 경계선 부분 제거(0으로 직선 긋기)
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


// 책 한권 한권의 이미지를 저장하기
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


// 직선들 중 세로선 추출
void BookImgPreProcessor::GetVLines(const vector<Vec4i>& inputLines, vector<Vec4i>& outputLines, const double verticalThreshold) {
	for (const auto& l : inputLines) {
		if (abs(getSlope(l)) >= verticalThreshold)
		{
			outputLines.push_back(l);
		}
	}
}


// 선분을 화면 전체로 연장
void BookImgPreProcessor::ExtendLineToMax(const Vec4i& inputLine, Vec4i& outputLine) {
	int x1 = inputLine[0], y1 = inputLine[1], x2 = inputLine[2], y2 = inputLine[3];
	double m = getSlope(inputLine);
	double b = y1 - m * x1;

	if (x1 == x2) { // 수직선
		outputLine[0] = x1;
		outputLine[1] = 0;
		outputLine[2] = x1;
		outputLine[3] = height;
	}
	else if (m == 0) { // 수평선
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
	// Hough Transform 파라미터
	constexpr double rho = 1; // distance resolution in pixels of the Hough grid
	constexpr double theta = CV_PI / 180; // angular resolution in radians of the Hough grid
	constexpr int houghThreshold = 100;	 // minimum number of votes(intersections in Hough grid cell)
	constexpr double minLineLength = 150; //minimum number of pixels making up a line
	constexpr double maxLineGap = 40;	//maximum gap in pixels between connectable line segments

	
	if (srcImg.empty()) {
		cerr << "[Error] Please set Image before excute run()" << endl;
		return;
	}



	// 흑백으로 변환
	//Mat gray;
	cvtColor(srcImg, grayImg, COLOR_RGB2GRAY);

	// 엣지 검출(Canny 엣지 검출기)
	Canny(grayImg, edgeImg, 50, 150);

	//짧은 엣지 제거
	removeShortEdges(200);

	// 허프 변환으로 직선 추출
	vector<Vec4i> lines;
	HoughLinesP(trimmedEdgeImg, lines, rho, theta, houghThreshold, minLineLength, maxLineGap);

	// 추출한 직선들중 세로선 추출(허용각도 Pi/2 +- Pi/15)
	vector<Vec4i> vLines;
	const double verticalThreshold = tan(CV_PI / 2 - CV_PI / 15);

	GetVLines(lines, vLines, verticalThreshold);

	// 선분을 화면에 꽉차게 연장
	size_t num_of_vertical_lines = vLines.size();
	for (size_t i = 0; i < num_of_vertical_lines; i++) {
		ExtendLineToMax(vLines[i], vLines[i]);
	}

	// x 좌표 기준으로 정렬(세로선들 사이의 갭을 계산하기 위해)
	auto cmpX = [](const Vec4i& a, const Vec4i& b) {
		if (a[0] != b[0])
			return a[0] < b[0];
		else
			return a[2] < b[2];
	};

	sort(vLines.begin(), vLines.end(), cmpX);

	// 세로선들 사이의 갭이 일정 값 이하이면 무시
	const int GapThreshold = cvRound(width / 50);
	//getBookImgsvector<Vec<Point, 4>> bookAreas;
	vector<Vec4i> boarderLines;

	findBookAreas(vLines, boarderLines, GapThreshold);

	// 책 표지 잘라내서 저장하기
	//saveBookCovers();

	// 사진 위에 결과 그리기
	resultImg = srcImg.clone();
	Scalar Red(0, 0, 255);

	fillBookAreas(resultImg, resultImg);
	drawLines(resultImg, resultImg, boarderLines, Red, 2);


	//waitKey();
	//destroyAllWindows();

}
