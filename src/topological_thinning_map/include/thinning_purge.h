
#ifndef THINNING_PURGE_H_
#define THINNING_PURGE_H_
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>  

void thinningIteration(cv::Mat& img, int iter);
void thinning(const cv::Mat& src, cv::Mat& dst);
void cut(cv::Mat& skeleton);
std::vector<cv::Point> findPixels(const cv::Mat& binaryImage);
std::vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis , unsigned int thresholdMax);
double distance(cv::Point p1, cv::Point p2);
cv::Mat eroded_dilated(cv::Mat& src);

std::vector<cv::Point> find_vertexs( cv::Mat& skeleton);

#endif