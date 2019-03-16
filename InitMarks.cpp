#include "InitMarks.h"
#include"Utils.h"
using namespace std;
using namespace cv;

//尾部摄像头：videoend40*
//求交界线的限定范围---40公分
//const int END_LEFT_LOW_40 = 400;
//const int END_LEFT_HIGH_40 = 520;
//const int END_RIGHT_LOW_40 = 430;
//const int END_RIGHT_HIGH_40 = 550;
//
////求交界位置或打标记在X方向上的范围-40公分位置
//const int END_LEFT_40 = 250;
//const int END_RIGHT_40 = 550;
////找交界位置在X方向上的中间位置
//const int END_TRUCK_MID_40 = (END_LEFT_40 + END_RIGHT_40) / 2;
//
////打标记在y方向上的范围---尾部摄像头
//const int END_LOW_40 = 70;
//const int END_HIGH_40 = 80;
//
////videoend20*
////求交界线的限定范围---20公分
//const int END_LEFT_LOW_20 = 250;
//const int END_LEFT_HIGH_20 = 320;
//const int END_RIGHT_LOW_20 = 280;
//const int END_RIGHT_HIGH_20 = 350;
//
////求交界位置或打标记在X方向上的范围-20公分位置
//const int END_LEFT_20 = 350;
//const int END_RIGHT_20 = 620;
////找交界位置在X方向上的中间位置
//const int END_TRUCK_MID_20 = (END_LEFT_20 + END_RIGHT_20) / 2;
//
////打标记在y方向上的范围
//const int END_LOW_20 = 70;
//const int END_HIGH_20 = 80;
//
//
////车头处摄像头
////40公分求交界线的限定范围
//const int HEAD_LEFT_LOW_40 = 400;
//const int HEAD_LEFT_HIGH_40 = 500;
//const int HEAD_RIGHT_LOW_40 = 350;
//const int HEAD_RIGHT_HIGH_40 = 450;
//
//
////求交界位置或打标记在X方向上的范围-40公分位置
//const int HEAD_LEFT_40 = 280;
//const int HEAD_RIGHT_40 = 600;
////找交界位置在X方向上的中间位置
//const int HEAD_TRUCK_MID_40 = (HEAD_LEFT_40 + HEAD_RIGHT_40) / 2;
////打标记在y方向上的范围-40公分位置
//const int HEAD_LOW_40 = 50;
//const int HEAD_HIGH_40 = 50;
//
//
////20公分求交界线的限定范围
//const int HEAD_LEFT_LOW_20 = 240;
//const int HEAD_LEFT_HIGH_20 = 330;
//const int HEAD_RIGHT_LOW_20 = 220;
//const int HEAD_RIGHT_HIGH_20 = 310;
//
////求交界位置或打标记在X方向上的范围-20公分位置
//const int HEAD_LEFT_20 = 240;
//const int HEAD_RIGHT_20 = 480;
////找交界位置在X方向上的中间位置
//const int HEAD_TRUCK_MID_20 = (HEAD_LEFT_20 + HEAD_RIGHT_20) / 2;
//
////Y方向打标记的范围----头部摄像头
//const int HEAD_LOW_20 = 50;
//const int HEAD_HIGH_20 = 40;
//
////侧边摄像头交界线的限定范围
//const int SIDE_LEFT_LOW_IN = 400;
//const int SIDE_LEFT_HIGH_IN = 720;
//const int SIDE_RIGHT_LOW_IN = 320;
//const int SIDE_RIGHT_HIGH_IN = 680;
//
////求交界位置或打标记在X方向上的范围-20公分位置
//const int SIDE_LEFT_IN = 200;
//const int SIDE_RIGHT_IN = 700;
////找交界位置在X方向上的中间位置
//const int SIDE_TRUCK_MID_IN = (SIDE_LEFT_IN + SIDE_RIGHT_IN) / 2;
//
////Y方向打标记的范围----头部摄像头
//const int SIDE_LOW_IN = 80;
//const int SIDE_HIGH_IN = 200;
//



InitMarks::InitMarks(int truck_head_end_flag, int truck_pos_flag)
{
	this->truck_head_end_flag = truck_head_end_flag;
	this->truck_pos_flag = truck_pos_flag;
	initPoinDownNumb = 0;
	initPoinUpNumb = 0;
	setMarkPos();
}

InitMarks::InitMarks()
{

}
InitMarks::~InitMarks()
{

}

cv::Mat InitMarks::getLsdPointsToLS(cv::Mat pointsLsd)
{
	Mat pointsToLS;
	MatIterator_<Vec4f> lineP_it = pointsLsd.begin<Vec4f>();
	
	Point2i signPosBeg(0, 0);
	Point2i signPosEnd(0, 0);
	int lineNum = 0;
	float kLine = 10;
	while (lineP_it != pointsLsd.end<Vec4f>())
	{
		float dx, dy;
		dx = ((*lineP_it)[2] - (*lineP_it)[0]);
		dy = ((*lineP_it)[3] - (*lineP_it)[1]);
		if (dx != 0)
		{
			kLine = dy / dx;
		}
		lineSlope.push_back(kLine);
		if (kLine>getMinSlope()&&kLine<getMaxSlope())
		{
			if (
				((*lineP_it)[0]<getBundaryXmid()&&
				(*lineP_it)[0]>getMarkPosLeft()&&
				(*lineP_it)[1] <=getBundaryLeftHigh()&&
				(*lineP_it)[1] >=getBundaryLeftLow())||
				((*lineP_it)[2] >= getBundaryXmid() &&
				((*lineP_it)[2] <= getMarkPosRight()) &&
				((*lineP_it)[3] <= getBundaryRightHigh()) &&
				((*lineP_it)[3] >= getBundaryRightLow()))
				)
			{
				pointsToLS.push_back(cv::Point2f((*lineP_it)[0], (*lineP_it)[1]));
				pointsToLS.push_back(cv::Point2f((*lineP_it)[2], (*lineP_it)[3]));
			}
		}
		lineP_it++;
	}

	return pointsToLS;
}
void InitMarks::markFirstFrame(LeastSquare &ls, cv::Mat &initialPoints, cv::Mat&pointsFlag, Mat&firstFrame)
{
	Mat frame;
	firstFrame.copyTo(frame);
	Size subPixWinSize(10, 10), winSize(31, 31);
	Mat gray;
	cvtColor(firstFrame, gray, COLOR_BGR2GRAY);
	Point2i pTemp(0, 0);
	int d = ((Utils::BOUND_RIGHT_HIGH - Utils::BOUND_RIGHT_LOW) + (Utils::BOUND_LEFT_HIGH - Utils::BOUND_LEFT_LOW)) / 2;

	for (int i = getMarkPosLeft(); i < getMarkPosRight();)
	{

		pTemp.x = i;
		double y = ls.getY(i);
		for (int j = y - getMarkPosLow(); j < y + getMarkPosHigh();)
		{
			pTemp.y = j;
			if (j <= y)
			{
				if(j<y- 0.5*d)
				{
					pointsFlag.push_back(2);
				}
				else
				{
					pointsFlag.push_back(1);

				}
				circle(firstFrame, pTemp, 4, Scalar(255, 0, 0), 1, 8);
				initPoinUpNumb++;
			}
			else
			{
				if (j > y + 0.4*d)
				{
					pointsFlag.push_back(-2);

				}
				else {
					pointsFlag.push_back(-1);

				}
				
				circle(firstFrame, pTemp, 4, Scalar(255, 0, 0), 1, 8);
				initPoinDownNumb++;
			}
			vector<Point2f> tmp;
			tmp.push_back(pTemp);
			initialPoints.push_back(tmp[0]);
			j += 5;
		}
		i += 10;
	}
	//line(frame, Point2i(END_TRUCK_MID_40, END_LEFT_LOW_40), Point2i(END_TRUCK_MID_40, END_LEFT_HIGH_40), Scalar(255, 0, 0), 2, 8);
	line(frame, Point2i(getMarkPosLeft(), getBundaryLeftLow()), Point2i(getMarkPosLeft(), getBundaryLeftHigh()), Scalar(255, 0, 0), 2, 8);
	line(frame, Point2i(getMarkPosRight(), getBundaryRightLow()), Point2i(getMarkPosRight(), getBundaryRightHigh()), Scalar(255, 0, 0), 2, 8);

	//显示第一帧标记的图片
	//imshow("5.获取标记点", firstFrame);
	//imwrite("5获取标记点.jpg", firstFrame);

}


void InitMarks::setMarkPos()
{

	//设置初始化数据中的斜率
	float k = (Utils::boundInit[0].end.y - Utils::boundInit[0].start.y) / (Utils::boundInit[0].end.x - Utils::boundInit[0].start.x);
	//cout << "init slope=" << k << endl;
	setMinSlope(k - 0.1);
	setMaxSlope(k + 0.1);

	//设置范围参数：
	if (Utils::boundInit[1].end.y < Utils::boundInit[1].start.y)
	{
		Utils::BOUND_LEFT_LOW = Utils::boundInit[1].end.y;
		Utils::BOUND_LEFT_HIGH = Utils::boundInit[1].start.y;
	}
	else
	{
		Utils::BOUND_LEFT_LOW = Utils::boundInit[1].start.y;
		Utils::BOUND_LEFT_HIGH = Utils::boundInit[1].end.y;
	}
	if (Utils::boundInit[2].end.y < Utils::boundInit[2].start.y)
	{
		Utils::BOUND_RIGHT_LOW = Utils::boundInit[2].end.y;
		Utils::BOUND_RIGHT_HIGH = Utils::boundInit[2].start.y;
	}
	else
	{
		Utils::BOUND_RIGHT_LOW = Utils::boundInit[2].start.y;
		Utils::BOUND_RIGHT_HIGH = Utils::boundInit[2].end.y;
	}
	
	
	Utils::MARK_LEFT = (Utils::boundInit[1].end.x + Utils::boundInit[1].start.x) / 2;
	Utils::MARK_RIGHT= (Utils::boundInit[2].end.x + Utils::boundInit[2].start.x) / 2;
	Utils::MARK_MID = (Utils::MARK_LEFT + Utils::MARK_RIGHT) / 2;
	int d = ((Utils::BOUND_RIGHT_HIGH - Utils::BOUND_RIGHT_LOW)+(Utils::BOUND_LEFT_HIGH- Utils::BOUND_LEFT_LOW))/2;
	Utils::MARK_LOW = 1.2*d;
	Utils::MARK_HIGH=1.3*d;
/*

	cout << "Utils::MARK_LEFT=" << Utils::MARK_LEFT << endl;
	cout << "Utils::MARK_RIGHT=" << Utils::MARK_RIGHT << endl;
	cout << "Utils::MARK_LOW=" << Utils::MARK_LOW << endl;
	cout << "Utils::MARK_HIGH=" << Utils::MARK_HIGH << endl;
	cout << "Utils::BOUND_LEFT_LOW=" << Utils::BOUND_LEFT_LOW << endl;
	cout << "Utils::BOUND_LEFT_HIGH=" << Utils::BOUND_LEFT_HIGH << endl;
	cout << "Utils::BOUND_RIGHT_LOW=" << Utils::BOUND_RIGHT_LOW << endl;
	cout << "Utils::BOUND_RIGHT_HIGH=" << Utils::BOUND_RIGHT_LOW << endl;
	cout << "Utils::MARK_MID=" << Utils::MARK_MID << endl;

*/
	setMarkPosLeft(Utils::MARK_LEFT);
	setMarkPosRight(Utils::MARK_RIGHT);
	setMarkPosLow(Utils::MARK_LOW);
	setMarkPosHigh(Utils::MARK_HIGH);
	setBundaryLeftLow(Utils::BOUND_LEFT_LOW);
	setBundaryLeftHigh(Utils::BOUND_LEFT_HIGH);
	setBundaryRightLow(Utils::BOUND_RIGHT_LOW);
	setBundaryRightHigh(Utils::BOUND_RIGHT_HIGH);
	setMarkXmid(Utils::MARK_MID);
}