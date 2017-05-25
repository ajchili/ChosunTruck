#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/photo/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Windows.h>
#include <iostream>
#include <string>
#include <chrono>
#include "ets2_self_driving.h"
#include "IPM.h"

#define PI 3.1415926

using namespace cv;
using namespace std;

void Thinning(Mat input, int row, int col);

void GetDesktopResolution(int& monitorWidth, int& monitorHeight) {
	RECT desktop;
	const HWND hDesktop = GetDesktopWindow();
	GetWindowRect(hDesktop, &desktop);
	monitorWidth = desktop.right;
	monitorHeight = desktop.bottom;
}

void GetGameResolution(int& width, int& height) {
	RECT windowsize;
	const HWND hWnd = FindWindow("prism3d", NULL);
	GetClientRect(hWnd, &windowsize);
	width = windowsize.right;
	height = windowsize.bottom;
}

void detectPause() {
	// Press '+' to pause
	if (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000)
	{
		while (true)
		{
			// Press '-' to start
			if (GetAsyncKeyState(VK_OEM_MINUS) & 0x8000)
			{
				break;
			}
		}
	}
}

int main() {
	int width = 0, height = 0;
	int monitorWidth = 0, monitorHeight = 0;
	long long int sum = 0;
	long long int i = 0;
	int diffOld = 0;
	GetGameResolution(width, height);
	double IPM_BOTTOM_RIGHT = width + 400;
	double IPM_BOTTOM_LEFT = -400;
	double IPM_RIGHT = width / 2 + 100;
	double IPM_LEFT = width / 2 - 100;
	int IPM_diff = 0;

	while (true)
	{
		detectPause();
		HWND hWnd = FindWindow("prism3d", NULL);
		HWND consoleWindow = GetConsoleWindow();
		GetDesktopResolution(monitorWidth, monitorHeight);

		Mat image, outputImg;
		hwnd2mat(hWnd).copyTo(image);

		medianBlur(image, image, 3);

		// The 4-points at the input image	
		vector<Point2f> origPoints;
		origPoints.push_back(Point2f(IPM_BOTTOM_LEFT, height - 50));
		origPoints.push_back(Point2f(IPM_BOTTOM_RIGHT, height - 50));
		origPoints.push_back(Point2f(IPM_RIGHT, height / 2 + 30));
		origPoints.push_back(Point2f(IPM_LEFT, height / 2 + 30));

		// The 4-points correspondences in the destination image
		vector<Point2f> dstPoints;
		dstPoints.push_back(Point2f(0, height));
		dstPoints.push_back(Point2f(width, height));
		dstPoints.push_back(Point2f(width, 0));
		dstPoints.push_back(Point2f(0, 0));

		// IPM object
		IPM ipm(Size(width, height), Size(width, height), origPoints, dstPoints);
		ipm.applyHomography(image, outputImg);

		cv::Mat gray;
		cv::Mat blur;
		cv::Mat sobel;
		cv::Mat contours;

		cv::resize(outputImg, outputImg, cv::Size(320, 240));
		cv::cvtColor(outputImg, gray, COLOR_RGB2GRAY);
		cv::blur(gray, blur, cv::Size(10, 10));
		cv::Sobel(blur, sobel, blur.depth(), 1, 0, 3, 0.5, 127);
		cv::threshold(sobel, contours, 145, 255, CV_THRESH_BINARY);

		LineFinder ld;
		ld.setLineLengthAndGap(20, 120);
		ld.setMinVote(55);

		std::vector<cv::Vec4i> li = ld.findLines(contours);
		ld.drawDetectedLines(contours);

		// Move and display the output windows
		imshow("Lines", contours);
		imshow("Road", outputImg);
		cv::moveWindow("Lines", monitorWidth / 1.6, monitorHeight / 10.8);
		cv::moveWindow("Road", monitorWidth / 1.2673, monitorHeight / 10.8);
		SetWindowPos(consoleWindow, 0, monitorWidth / 1.6, monitorHeight / 2.7, 600, 400, SWP_NOZORDER);
		SetWindowPos(hWnd, 0, 0, 0, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
		waitKey(1);

		SetActiveWindow(hWnd);
		POINT pt;
		GetCursorPos(&pt);
		cout << "current mouse pos: " << "x: " << pt.x << "y: " << pt.y << endl;

		int bottom_center = 160;
		int sum_centerline = 0;
		int count_centerline = 0;
		int first_centerline = 0;
		int last_centerline = 0;
		double avr_center_to_left = 0;
		double avr_center_to_right = 0;

		//#pragma omp parallel for
		for (int i = 240; i > 30; i--)
		{
			double center_to_right = -1;
			double center_to_left = -1;

			for (int j = 0; j < 150; j++)
			{
				if (contours.at<uchar>(i, bottom_center + j) == 112 && center_to_right == -1)
				{
					center_to_right = j;
				}
				if (contours.at<uchar>(i, bottom_center - j) == 112 && center_to_left == -1)
				{
					center_to_left = j;
				}
			}
			if (center_to_left != -1 && center_to_right != -1)
			{
				int centerline = (center_to_right - center_to_left + 2 * bottom_center) / 2;
				if (first_centerline == 0)
				{
					first_centerline = centerline;
				}
				cv::circle(outputImg, Point(centerline, i), 1, Scalar(30, 255, 30), 3);
				cv::circle(outputImg, Point(centerline + center_to_right + 20, i), 1, Scalar(255, 30, 30), 3);
				cv::circle(outputImg, Point(centerline - center_to_left + 10, i), 1, Scalar(255, 30, 30), 3);
				sum_centerline += centerline;
				avr_center_to_left = (avr_center_to_left * count_centerline + center_to_left) / count_centerline + 1;
				avr_center_to_right = (avr_center_to_right * count_centerline + center_to_right) / count_centerline + 1;
				last_centerline = centerline;
				count_centerline++;
			} else {}
		}

		pt.x = width / 2;
		if (count_centerline != 0)
		{
			int diff = sum_centerline / count_centerline - bottom_center - 25;

			// diff_max was determined by finding the maxmimum diff that can be used to go from center to the very edge of the lane.
			int diff_max = 70;

			// jerk_factor = how fast the wheel will turn
			// (1/70) = Limits steering to move 1px MAXMIMUM every time step (1 second).
			double jerk_factor = 1 / 70;

			// diff on a scale of -1 to 1
			double linearized_diff = diff / diff_max;

			double turn_amount = linearized_diff * jerk_factor;

			if (turn_amount < .5)
			{
				turn_amount = 0;
			} else
			{
				turn_amount = 1;
			}

			int moveMouse = (pt.x + diffOld + turn_amount);
			SetCursorPos(moveMouse, height / 2);
			cout << "Steer: " << diffOld << "px " << endl;

			// Compensate movement for IPM display
			/*double diffForIPM = (diff - diffOld) / 3;
			if ((int)diffForIPM == 0) {
				if (IPM_diff > 0) {
					IPM_RIGHT -= 1;
					IPM_LEFT -= 1;
					IPM_diff -= 1;
				}
				else if (IPM_diff < 0) {
					IPM_RIGHT += 1;
					IPM_LEFT += 1;
					IPM_diff += 1;
				}
				else {
					IPM_RIGHT = width / 2 + 100;
					IPM_LEFT = width / 2 - 100;
					IPM_diff = 0;
				}
			}
			else {
				if (IPM_diff >= -30 && IPM_diff <= 30) {

					if ((int)diffForIPM > 0) {
						IPM_RIGHT += (int)diffForIPM;
						IPM_LEFT += (int)diffForIPM;
						IPM_diff++;
					}
					else {
						IPM_RIGHT -= (int)diffForIPM;
						IPM_LEFT -= (int)diffForIPM;
						IPM_diff--;
					}
				}
			}*/
			//cout << IPM_diff <<" / " << (int)diffForIPM << endl;
			diffOld = diff;
		}
	}
	return 0;
}
