#ifndef __ets_self_driving_h__
#define __ets_self_driving_h__

#include <opencv2/opencv.hpp>

#include <Windows.h>

#define PI 3.1415926

using namespace std;

class LineFinder
{
private:
	cv::Mat image; // �� ����
	std::vector<cv::Vec4i> lines; // ���� �����ϱ� ���� ������ ���� ������ ����
	double deltaRho;
	double deltaTheta; // ������ �ػ��� �Ķ�����
	int minVote; // ���� �����ϱ� ���� �޾ƾ� �ϴ� �ּ� ��ǥ ����
	double minLength; // ���� ���� �ּ� ����
	double maxGap; // ���� ���� �ִ� ���� ����

public:
	LineFinder::LineFinder() : deltaRho(1), deltaTheta(PI / 180), minVote(50), minLength(50), maxGap(10) {}
	// �⺻ ���� �ػ󵵴� 1���� 1ȭ�� 
	// ������ ���� �ּ� ���̵� ����
	void setAccResolution(double dRho, double dTheta);
	void setMinVote(int minv);
	void setLineLengthAndGap(double length, double gap);
	std::vector<cv::Vec4i> findLines(cv::Mat& binary);
	void drawDetectedLines(cv::Mat& image, cv::Scalar color = cv::Scalar(112, 112, 0));
};

cv::Mat hwnd2mat(HWND hwnd);
void cudaf();

#endif
