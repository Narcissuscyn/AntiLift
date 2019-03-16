#include "PreProcessPoints.h"
#include <iostream>
using namespace std;
using namespace cv;
#include <fstream>

PreProcessPoints::PreProcessPoints()
{
}


PreProcessPoints::~PreProcessPoints()
{
}
void PreProcessPoints::clearPoints(cv::Mat &initialPoints, cv::Mat &endPoints, cv::Mat&pointsFlag)
{
	int k = 0;
	float averBoxDy = 0, averTrucDx = 0;
	float averBoxDx = 0, averTrucDy = 0;
	int averBoxNum = 0, averTrucNum = 0;
	for (int i = 0; i < initialPoints.rows; i++)
	{
		float dx, dy, x, y;
		y = initialPoints.at<cv::Point2f>(i).y - endPoints.at<cv::Point2f>(i).y;
		x = initialPoints.at<cv::Point2f>(i).x - endPoints.at<cv::Point2f>(i).x;
		dy = abs(y);
		dx = abs(x);

		if (pointsFlag.at<int>(i) == 2)
		{
			averBoxDx += dx;
			averBoxDy += dy;
			averBoxNum++;
		}
		else if (pointsFlag.at<int>(i) == -2)
		{
			averTrucDx += dx;
			averTrucDy += dy;
			averTrucNum++;
		}
	}

	averBoxDx /= averBoxNum;
	averBoxDy /= averBoxNum;
	averTrucDx /= averTrucNum;
	averTrucDy /= averTrucNum;
	k = 0;
	for (int i = 0;i < initialPoints.rows;i++)
	{
		float dx, dy, x, y;
		y = initialPoints.at<cv::Point2f>(i).y - endPoints.at<cv::Point2f>(i).y;
		x = initialPoints.at<cv::Point2f>(i).x - endPoints.at<cv::Point2f>(i).x;
		if (y < -3)
		{
			continue;
		}
		dy = abs(y);
		dx = abs(x);
		if (pointsFlag.at<int>(i) == 2)
		{
			if (dy > 3 * averBoxDy || dx > 4 * averBoxDx)
			{
				continue;
			}
			if (dx < (averBoxDx / 3))
			{
				continue;
			}
		}
		else if (pointsFlag.at<int>(i) == -2)
		{
			if (dy > 4 * averTrucDy || dx > 4 * averTrucDx)
			{
				continue;
			}
			if (dy < (averTrucDy / 3) || dx < (averTrucDx / 4))
			{
				continue;
			}
		}
		initialPoints.at<cv::Point2f>(k) = initialPoints.at<cv::Point2f>(i);
		endPoints.at<cv::Point2f>(k) = endPoints.at<cv::Point2f>(i);
		pointsFlag.at<int>(k) = pointsFlag.at<int>(i);
		k++;
	}
	initialPoints.resize(k);
	endPoints.resize(k);
	pointsFlag.resize(k);
}

void PreProcessPoints::distinguishPoints(
	cv::Mat &initialPoints,
	cv::Mat &endPoints,
	cv::Mat&pointsFlag,
	cv::Mat &lineUp,
	cv::Mat &lineDown,
	cv::Mat &initDown,
	cv::Mat &initUp,
	cv::Mat &whiteFrame
)
{

	for (int i = 0; i < endPoints.rows; i++)
	{
		if (pointsFlag.at<int>(i) > 1)
		{
			lineUp.push_back(endPoints.at<Point2f>(i));
			initUp.push_back(initialPoints.at<Point2f>(i));
			circle(whiteFrame, endPoints.at<Point2f>(i), 2, Scalar(0, 0, 255), 5, 8);
		}
		else if (pointsFlag.at<int>(i) < -1)
		{
			lineDown.push_back(endPoints.at<Point2f>(i));
			initDown.push_back(initialPoints.at<Point2f>(i));
			circle(whiteFrame, initialPoints.at<Point2f>(i), 2, Scalar(255, 255, 255), 5, 8);
			circle(whiteFrame, endPoints.at<Point2f>(i), 2, Scalar(255, 0, 0), 5, 8);
		}
		else
		{
			circle(whiteFrame, endPoints.at<Point2f>(i), 2, Scalar(0, 255, 0), 5, 8);
		}
	}


	//imshow("6.预处理判断点集", whiteFrame);
	//imwrite("6预处理判断点集.jpg", whiteFrame);
}