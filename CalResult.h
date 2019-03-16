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
	//1）车上标记在Y方向上运动的判断：如果几乎所有的点都有较大运动，则发出警报!
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
	//2)车上标记在X方向上的运动判断：
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
	//3)集装箱上的标记点在x方向上的运动判断
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

	//直线拟合比较
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

