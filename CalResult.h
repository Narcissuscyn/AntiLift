#pragma once
#include <opencv.hpp>
#include <vector>

class CalResult
{
public:
	//************************************
	// Method:    CalResult
	// FullName:  CalResult::CalResult
	// Access:    public 
	// Returns:   
	// Qualifier:
	// Parameter: std::vector<cv::Point2f> lineDown
	// Parameter: std::vector<cv::Point2f>initDown
	// Parameter: std::vector<cv::Point2f> lineUp
	// Parameter: std::vector<cv::Point2f>initUp
	//************************************
	CalResult(cv::Mat lineDown1, cv::Mat initDown1, cv::Mat lineUp1, cv::Mat initUp1, int initPoinDownNumb,int initPoinUpNumb);
	/*
	//1�����ϱ����Y�������˶����жϣ�����������еĵ㶼�нϴ��˶����򷢳�����!
	*/
	//************************************
	// Method:    calPointTruckDirY
	// FullName:  CalResult::calPointTruckDirY
	// Access:    public 
	// Returns:   int
	// Qualifier:
	// Parameter: cv::Mat & frame
	//************************************
	int calPointTruckDirY();
	/*
	//2)���ϱ����X�����ϵ��˶��жϣ�
	*/
	//************************************
	// Method:    calPointTruckDirX
	// FullName:  CalResult::calPointTruckDirX
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: cv::Mat & frame
	//************************************
	bool calPointTruckDirX();
	/*
	//3)��װ���ϵı�ǵ���x�����ϵ��˶��ж�
	*/
	////************************************
	//// Method:    calPointBoxDirX
	//// FullName:  CalResult::calPointBoxDirX
	//// Access:    public 
	//// Returns:   bool
	//// Qualifier:
	//// Parameter: cv::Mat & frame
	////************************************
	//bool calPointBoxDirX();

	
	//************************************
	// Method:    calPointBoxSize
	// FullName:  CalResult::calPointBoxSize
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	//************************************
	bool calPointBoxSize();

	//************************************
	// Method:    calPointTruckSize
	// FullName:  CalResult::calPointTruckSize
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	//************************************
	/*
	return:
	true-points on the truck are very small
	false-points on the truck are very much
	*/
	bool calPointTruckSize();

	//ֱ����ϱȽ�
	bool judgeLS(cv::Mat &frame);


	//************************************
	// Method:    getResults
	// FullName:  CalResult::getResults
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: cv::Mat & frame
	//************************************
	void getResults(cv::Mat &frame);

	~CalResult();
	bool getIsCurrFramAlert() const { return isAlert; }
	void setIsAlert(bool val) { isAlert = val; }

	void getResultSide(cv::Mat &frame);

	void setDxDanger(int dx)
	{
		DX_DANGER = dx;
	}
	void setDyDanger(int dy)
	{
		DY_DANGER = dy;
	}

private:
	cv::Mat markSafe;
	cv::Mat markDanger;
	bool isAlert;
	cv::Mat lineDown;
	cv::Mat lineUp;

	cv::Mat initDown;
	cv::Mat initUp;

	int initPoinDownNumb;
	int initPoinUpNumb;

	int DX_DANGER;
	int DY_DANGER;
};

