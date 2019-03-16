#pragma once
#include <opencv.hpp>
class PreProcessPoints
{
public:
	PreProcessPoints();
	~PreProcessPoints();

	//************************************
	// Method:    clearPoints
	// FullName:  Utils::clearPoints
	// Access:    public static 
	// Returns:   void
	// Qualifier:
	// Parameter: cv::Mat & initialPoints
	// Parameter: cv::Mat & endPoints
	// Parameter: cv::Mat & pointFlag
	//************************************
	static void clearPoints(cv::Mat &initialPoints,cv::Mat&endPoints, cv::Mat&pointFlag);

	//************************************
	// Method:    distinguishPoints
	// FullName:  Utils::distinguishPoints
	// Access:    public static 
	// Returns:   void
	// Qualifier:
	// Parameter: cv::Mat & initialPoints
	// Parameter: cv::Mat & endPoints
	// Parameter: std::vector<int> & pointFlag
	// Parameter: cv::Mat & lineUp
	// Parameter: cv::Mat & lineDown
	// Parameter: cv::Mat & initDown
	// Parameter: cv::Mat & initUp
	// Parameter: cv::Mat & whiteFrame
	// Parameter: int & end_poin_down_numb
	//************************************
	static void distinguishPoints(
		cv::Mat &initialPoints,
		cv::Mat &endPoints,
		cv::Mat&pointsFlag,
		cv::Mat &lineUp,
		cv::Mat &lineDown,
		cv::Mat &initDown,
		cv::Mat &initUp,
		cv::Mat &whiteFrame
	);
};

