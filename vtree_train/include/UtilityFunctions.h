#pragma once
#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <vector>
#include <opencv2/opencv.hpp>

double Distance(double x1, double y1, double x2, double y2);
double fitAngleInRange(double angle) ;
void sleep(int m);

double L1Norm(const std::vector<double>& input);
double L2Norm(const std::vector<float>& input);
void MatTo1DVector(const cv::Mat& src, std::vector<unsigned char>& output);
void MatTo2DVector(const cv::Mat& src, std::vector<std::vector<unsigned char>>& output);
void resizeIfNecessary(const cv::Mat& input, cv::Mat& output, int minimumLength = 200, int maximumLength = 1500);
std::size_t hammigDistance(const std::vector<uchar>& data1, const std::vector<uchar>& data2);

#endif
