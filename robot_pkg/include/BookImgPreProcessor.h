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
	void saveBookCovers(const std::string& filenamePrefix); // å �ѱ� �ѱ��� �̹����� �����ϱ�

private:
	cv::Mat srcImg;
	cv::Mat edgeImg;
	cv::Mat trimmedEdgeImg;
	cv::Mat resultImg;
	std::vector<cv::Vec<cv::Point, 4>> bookAreas;


	int width;
	int height;

	// ª�� ������ ����
	void removeShortEdges(const int threshold);

	// ������ ���� ��ȯ
	double getSlope(const cv::Vec4i& line);

	// �� ������ �������� Ȯ��
	bool isParallel(const cv::Vec4i& line1, const cv::Vec4i& line2);

	// �� ������ ȭ�� ������ ������ ������ Ȯ��
	bool crossOnScreen(const cv::Vec4i& line1, const cv::Vec4i& line2);

	// �̹��� ���� ���� �������� �ѹ��� �׸���
	void drawLines(const cv::Mat& inputImg, cv::Mat& outputImg, const std::vector<cv::Vec4i>& lines, const cv::Scalar color, const int thickness);

	// �� �� ������ �Ÿ�(L2 norm)
	double Distance(const cv::Point& A, const cv::Point& B);

	// 2���� ���μ��� ���� ����
	int GapBetweenVLines(const cv::Vec4i& line1, const cv::Vec4i& line2);

	// å�� ���� �� �ִ� ���� ã��
	// �ݵ�� ����(x��ǥ ����)�� ���������� ����� ��!!!
	void findBookAreas(const std::vector<cv::Vec4i>& sortedVLines, std::vector<cv::Vec4i>& boarderLines, const int GapThreshold);



	// å ������ �ʷϻ����� ��ĥ
	void fillBookAreas(const cv::Mat& inputImg, cv::Mat& outputImg);

	// ������ �� ���μ� ����
	void GetVLines(const std::vector<cv::Vec4i>& inputLines, std::vector<cv::Vec4i>& outputLines, const double verticalThreshold);

	// ������ ȭ�� ��ü�� ����
	void ExtendLineToMax(const cv::Vec4i& inputLine, cv::Vec4i& outputLine);



};


#endif // !BOOKSEGMENTATION_H
