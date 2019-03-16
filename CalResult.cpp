#include "CalResult.h"
#include "Utils.h"
#include "LeastSquare.h"
#include <fstream>
using namespace cv;
using namespace std;

//将要分析的点集最小点的个数；

const int MIN_LS_SIZE = 10;
//const int DX_DANGER = 8;
//const int DY_DANGER = 12;
//const int DX_DANGER = 20;
//const int DY_DANGER = 120;
Point2i water_mark_pos(10, 150);
const float STAND_SLOP_DIFF = 0.080;

CalResult::CalResult(cv::Mat lineDown1, cv::Mat initDown1, cv::Mat lineUp1, cv::Mat initUp1, int initPoinDownNumb1, int initPoinUpNumb1)
{
	markSafe = imread("logo_safe.jpg");
	markDanger = imread("logo_danger.jpg");
	lineDown = lineDown1;
	initDown = initDown1;
	lineUp = lineUp1;
	initUp = initUp1;
	initPoinUpNumb = initPoinUpNumb1;
	initPoinDownNumb = initPoinDownNumb1;

	setIsAlert(false);
}


CalResult::~CalResult()
{

}

bool CalResult::calPointBoxSize()
{
	
	if ((lineUp.rows < (initPoinUpNumb / 15)))//集装箱上的点太少
	{
		return false;
	}
	return true;
}
bool CalResult::calPointTruckSize()
{
	if ((lineDown.rows < initPoinDownNumb / 15))//车上的点太少
	{
		return false;
	}
	return true;
}
bool CalResult::calPointTruckDirX()
{
	int dynamicPointNumX = 0;
	for (int i = 0; i < lineDown.rows; i++)
	{
		float dx = lineDown.at<Point2f>(i).x - initDown.at<Point2f>(i).x;
		if (abs(dx) > DX_DANGER)
		{
			dynamicPointNumX++;
			continue;
		}
	}
	if (dynamicPointNumX > lineDown.rows / 4)
	{
		return true;
	}
	return false;
}
int CalResult::calPointTruckDirY()
{

	//ofstream outFile("lineDownMove.txt", ios::out);
	int safeNumY = 0, dangerNumY = 0;
	for (int i = 0; i < lineDown.rows; i++)
	{
		float dy = lineDown.at<Point2f>(i).y - initDown.at<Point2f>(i).y;
		if (abs(dy) > DY_DANGER)
		{
			dangerNumY++;
		}
		else
		{
			safeNumY++;
		}
	}
	if (safeNumY < (lineDown.rows / 5))//车上所有的点在Y方向都有较大运动
	{
		return 1;
	}
	else if (dangerNumY > (lineDown.rows /4))////车的一角的在Y方向有较大运动
	{
		return 2;
	}
	return 0;
}

bool CalResult::judgeLS(cv::Mat &frame)
{
	//init points on the truck
	Point2i signPosBeg(0, 0);
	Point2i signPosEnd(0, 0);
	cv::Mat mInitDown(initDown);
	LeastSquare truckinitls(mInitDown);
	//truckinitls.print();
	signPosBeg.x = 50;
	signPosBeg.y = truckinitls.getY(signPosBeg.x);
	signPosEnd.x = 1300;
	signPosEnd.y = truckinitls.getY(signPosEnd.x);
	line(frame, signPosBeg, signPosEnd, Scalar(255, 0, 0), 2, 8);
	//current points on the truck
	cv::Mat mLineDown(lineDown);
	LeastSquare truckendls(mLineDown);
	//truckendls.print();
	signPosBeg.x = 50;
	signPosBeg.y = truckendls.getY(signPosBeg.x);
	signPosEnd.x = 1300;
	signPosEnd.y = truckendls.getY(signPosEnd.x);
	line(frame, signPosBeg, signPosEnd, Scalar(255, 0, 0), 2, 8);

	//current points on the box
	cv::Mat mLineUp(lineUp);
	LeastSquare boxendls(mLineUp);
	//boxendls.print();
	signPosBeg.x = 50;
	signPosBeg.y = boxendls.getY(signPosBeg.x);
	signPosEnd.x = 1300;
	signPosEnd.y = boxendls.getY(signPosEnd.x);
	line(frame, signPosBeg, signPosEnd, Scalar(0, 255, 0), 2, 8);
	//analyse the relationships of three lines
	//1.two lines on the truck
	float d_truck = truckendls.getA() - truckinitls.getA();

	//2.two lines on the truck and box
	float d = boxendls.getA() - truckendls.getA();

	if (abs(d) > STAND_SLOP_DIFF&&abs(d_truck) > STAND_SLOP_DIFF)
	{
		return true;
	}
	return false;
}
void CalResult::getResultSide(cv::Mat &frame)
{
	//Utils::putString(frame, water_mark_pos, "Judging...............");
	int  isTruckDirY = calPointTruckDirY();
	bool isTruckSize = calPointTruckSize();
	water_mark_pos.y = 150;
	if (isTruckDirY == 2 && isTruckSize)
	{
		//Utils::putString(frame, water_mark_pos, "One corner of truck moved sharply in Y direction");
		//Utils::putWaterMark(frame, markDanger);
		setIsAlert(true);
		return;
	}	
//	Utils::putWaterMark(frame, markSafe);
}
void CalResult::getResults(cv::Mat &frame)
{


	//Utils::putString(frame, water_mark_pos, "Judging...............");

	bool isBoxSize = calPointBoxSize();
	bool isTruckSize = calPointTruckSize();
	bool isTruckDirX = calPointTruckDirX();
	int  isTruckDirY = calPointTruckDirY();

	if (isTruckSize&&isTruckDirY == 1)
	{
		//Utils::putWaterMark(frame, markDanger);
		//Utils::putString(frame, water_mark_pos, "Almost every point moved sharply in Y direction");
		setIsAlert(true);
		//water_mark_pos.y = 150;
		return;
	}

	bool isLS = false;
	if (isBoxSize&&isTruckSize&&initDown.rows > MIN_LS_SIZE&&lineDown.rows > MIN_LS_SIZE&&lineUp.rows > MIN_LS_SIZE)
	{
		isLS = judgeLS(frame);
	}

	/*if (isTruckDirY == 2 && isTruckSize)
	{
		Utils::putString(frame, water_mark_pos, "One corner of truck moved sharply in Y direction");
	}
	if (isTruckDirX)
	{
		Utils::putString(frame, water_mark_pos, "Points in the truck moved sharply in X direction");
	}
	if (isLS)
	{
		Utils::putString(frame, water_mark_pos, "Fitting a big Angle between two straight lines");
	}
*/
	water_mark_pos.y = 150;

	int alertCounts = 0;
	if (isTruckDirY && isTruckSize)
	{
		alertCounts++;
	}
	if (isTruckDirX)
	{
		alertCounts++;
	}
	if (isLS)
	{
		alertCounts++;
	}

	//cout << "alertCounts:" << alertCounts << endl;
	if (alertCounts >= 2)
	{
		//Utils::putWaterMark(frame, markDanger);
		setIsAlert(true);
		return;
	}

	//Utils::putWaterMark(frame, markSafe);
}
