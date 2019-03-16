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
	����ͨ��LSDֱ�߼�ⷽ���õ��ľ�������ĵ㼯��������Щ�㼯������С���˷���Ͻ�����λ��
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
	/*����λ���źţ�40Ϊ12m�ģ�20Ϊ6m�ģ�
	*/
	int truck_pos_flag;
	//ͷ������ͷ����β������ͷ��1----β������ͷ��2-ͷ������ͷ
	int truck_head_end_flag;

	//���ǵ��������ҵķ�Χ
	int markPosLeft;
	int markPosRight;
	int markPosLow;
	int markPosHigh;

	//�󽻽�λ��ʱ��ֱ�߱����������귶Χ��
	int bundaryLeftLow;
	int bundaryLeftHigh;
	int bundaryRightLow;
	int bundaryRightHigh;
	//X�����Ͻ����ߵ��м�λ��
	int bundaryXmid;
	
	//б�ʵ��Χ��
	float minSlope;
	float maxSlope;

	int initPoinDownNumb;
	int initPoinUpNumb;

	cv::Mat lineSlope;
};

