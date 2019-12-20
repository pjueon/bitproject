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
	void saveBookCovers(const std::string& filenamePrefix); 

private:
	cv::Mat srcImg;
	cv::Mat grayImg;
	cv::Mat edgeImg;
	cv::Mat trimmedEdgeImg;
	cv::Mat resultImg;
	std::vector<cv::Vec<cv::Point, 4>> bookAreas;

	int width;
	int height;

	void removeShortEdges(const int threshold);

	double getSlope(const cv::Vec4i& line);

	bool isParallel(const cv::Vec4i& line1, const cv::Vec4i& line2);

	bool crossOnScreen(const cv::Vec4i& line1, const cv::Vec4i& line2);

	void drawLines(const cv::Mat& inputImg, cv::Mat& outputImg, const std::vector<cv::Vec4i>& lines, const cv::Scalar color, const int thickness);

	double Distance(const cv::Point& A, const cv::Point& B);

	int GapBetweenVLines(const cv::Vec4i& line1, const cv::Vec4i& line2);

	void findBookAreas(const std::vector<cv::Vec4i>& sortedVLines, std::vector<cv::Vec4i>& boarderLines, const int GapThreshold);

	void fillBookAreas(const cv::Mat& inputImg, cv::Mat& outputImg);

	void GetVLines(const std::vector<cv::Vec4i>& inputLines, std::vector<cv::Vec4i>& outputLines, const double verticalThreshold);

	void ExtendLineToMax(const cv::Vec4i& inputLine, cv::Vec4i& outputLine);



};


#endif // !BOOKSEGMENTATION_H
