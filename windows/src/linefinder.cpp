#include "ets2_self_driving.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/photo/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>

#define PI 3.1415926

cv::Point prev_point;

using namespace cv;
using namespace std;

void LineFinder::setAccResolution(double dRho, double dTheta) {
	deltaRho = dRho;
	deltaTheta = dTheta;
}

void LineFinder::setMinVote(int minv) {
	minVote = minv;
}

void LineFinder::setLineLengthAndGap(double length, double gap) {
	minLength = length;
	maxGap = gap;
}

std::vector<cv::Vec4i> LineFinder::findLines(cv::Mat& binary) {
	lines.clear();
	cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
	return lines;
}

void LineFinder::drawDetectedLines(cv::Mat& image, cv::Scalar color) {
	std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();

	while (it2 != lines.end())
	{
		cv::Point startPoint((*it2)[0], (*it2)[1]);
		cv::Point endPoint = cv::Point((*it2)[2], (*it2)[3]);
		cv::line(image, startPoint, endPoint, color, 3);
		++it2;
	}
}
