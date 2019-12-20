#pragma once
#ifndef BOOK_PREPROCESSOR_H
#define BOOK_PREPROCESSOR_H

#include "opencv2/opencv.hpp"
#include <vector>
#include <string>


class BookImgPreProcessor {
public:
	BookImgPreProcessor();
	~BookImgPreProcessor() = default;

	void setImg(const cv::Mat& img);
	void setImg(const std::string& filename);
	void run();
	void showResult();
	void saveResult(const std::string& filenamePrefix);
	std::vector<cv::Mat> getBookImgs();
	void saveBookCovers(const std::string& filenamePrefix); // 책 한권 한권의 이미지를 저장하기

private:
	cv::Mat srcImg;
	cv::Mat edgeImg;
	cv::Mat trimmedEdgeImg;
	cv::Mat resultImg;
	std::vector<cv::Vec<cv::Point, 4>> bookAreas;


	int width;
	int height;

	// 짧은 엣지들 삭제
	void removeShortEdges(const int threshold);

	// 직선의 기울기 반환
	double getSlope(const cv::Vec4i& line);

	// 두 직선이 평행한지 확인
	bool isParallel(const cv::Vec4i& line1, const cv::Vec4i& line2);

	// 두 직선이 화면 내에서 교점을 갖는지 확인
	bool crossOnScreen(const cv::Vec4i& line1, const cv::Vec4i& line2);

	// 이미지 위에 직선 여러개를 한번에 그리기
	void drawLines(const cv::Mat& inputImg, cv::Mat& outputImg, const std::vector<cv::Vec4i>& lines, const cv::Scalar color, const int thickness);

	// 두 점 사이의 거리(L2 norm)
	double Distance(const cv::Point& A, const cv::Point& B);

	// 2개의 세로선들 사이 간격
	int GapBetweenVLines(const cv::Vec4i& line1, const cv::Vec4i& line2);

	// 책이 있을 수 있는 영역 찾기
	// 반드시 정렬(x좌표 기준)된 수직선들을 사용할 것!!!
	void findBookAreas(const std::vector<cv::Vec4i>& sortedVLines, std::vector<cv::Vec4i>& boarderLines, const int GapThreshold);



	// 책 영역을 초록색으로 색칠
	void fillBookAreas(const cv::Mat& inputImg, cv::Mat& outputImg);

	// 직선들 중 세로선 추출
	void GetVLines(const std::vector<cv::Vec4i>& inputLines, std::vector<cv::Vec4i>& outputLines, const double verticalThreshold);

	// 선분을 화면 전체로 연장
	void ExtendLineToMax(const cv::Vec4i& inputLine, cv::Vec4i& outputLine);



};


#endif // !BOOKSEGMENTATION_H
