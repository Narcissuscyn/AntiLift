#pragma once
#include <vector>
#include<opencv.hpp>
#include "LeastSquare.h"
#include <iostream>
class InitMarks
{
public:
	InitMarks();
	InitMarks(int truck_head_end_flag, int truck_pos_flag);
	/*
	返回通过LSD直线检测方法得到的经过处理的点集，将用这些点集进行最小二乘法拟合交界线位置
	*/
	cv::Mat getLsdPointsToLS(cv::Mat pointsLsd);


	//************************************
	// Method:    markFirstFrame
	// FullName:  InitMarks::markFirstFrame
	// Access:    public 
	// Returns:   void
	// Qualifier:
	//************************************
	void markFirstFrame(LeastSquare &ls, cv::Mat &initialPoints, cv::Mat&pointFlag, cv::Mat&firstFrame);
	void setMarkPos();

	~InitMarks();

	int getInitPoinDownNumb() const { return initPoinDownNumb; }
	void setInitPoinDownNumb(int val) { initPoinDownNumb = val; }
	int getMarkPosLeft() const { return markPosLeft; }
	void setMarkPosLeft(int val) { markPosLeft = val; }
	int getMarkPosRight() const { return markPosRight; }
	void setMarkPosRight(int val) { markPosRight = val; }
	int getMarkPosLow() const { return markPosLow; }
	void setMarkPosLow(int val) { markPosLow = val; }
	int getMarkPosHigh() const { return markPosHigh; }
	void setMarkPosHigh(int val) { markPosHigh = val; }

	int getInitPoinUpNumb() const { return initPoinUpNumb; }
	void setInitPoinUpNumb(int val) { initPoinUpNumb = val; }
	cv::Mat getLineSlope() const { return lineSlope; }
	void setLineSlope(cv::Mat val) { lineSlope = val; }
	int getBundaryLeftLow() const { return bundaryLeftLow; }
	void setBundaryLeftLow(int val) { bundaryLeftLow = val; }
	int getBundaryLeftHigh() const { return bundaryLeftHigh; }
	void setBundaryLeftHigh(int val) { bundaryLeftHigh = val; }
	int getBundaryRightLow() const { return bundaryRightLow; }
	void setBundaryRightLow(int val) { bundaryRightLow = val; }
	int getBundaryRightHigh() const { return bundaryRightHigh; }
	void setBundaryRightHigh(int val) { bundaryRightHigh = val; }
	int getBundaryXmid() const { return bundaryXmid; }
	void setMarkXmid(int val) { bundaryXmid = val; }
	float getMinSlope() const { return minSlope; }
	void setMinSlope(float val) { minSlope = val; }
	float getMaxSlope() const { return maxSlope; }
	void setMaxSlope(float val) { maxSlope = val; }
private:
	/*车的位置信号：40为12m的，20为6m的；
	*/
	int truck_pos_flag;
	//头部摄像头还是尾部摄像头：1----尾部摄像头；2-头部摄像头
	int truck_head_end_flag;

	//打标记的上下左右的范围
	int markPosLeft;
	int markPosRight;
	int markPosLow;
	int markPosHigh;

	//求交界位置时，直线被保留的坐标范围：
	int bundaryLeftLow;
	int bundaryLeftHigh;
	int bundaryRightLow;
	int bundaryRightHigh;
	//X方向上交界线的中间位置
	int bundaryXmid;
	
	//斜率的最范围：
	float minSlope;
	float maxSlope;

	int initPoinDownNumb;
	int initPoinUpNumb;

	cv::Mat lineSlope;
};

