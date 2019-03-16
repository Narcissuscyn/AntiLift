#pragma once
#include <opencv.hpp>
#include <string>
#include <vector>
struct Line
{
	cv::Point2d start;
	cv::Point2d end;
};

class Utils
{
public:
	Utils();
	~Utils();

	//************************************
	// Method:    putWaterMark
	// FullName:  Utils::putWaterMark
	// Access:    public static 
	// Returns:   cv::Mat
	// Qualifier:
	// Parameter: cv::Mat & image
	// Parameter: cv::Mat & logo
	// Parameter: cv::Point pos
	//************************************
	static cv::Mat putWaterMark(cv::Mat &image, cv::Mat &logo, cv::Point pos = cv::Point(10, 10));
	//************************************
	// Method:    putString
	// FullName:  Utils::putString
	// Access:    public static 
	// Returns:   void
	// Qualifier:
	// Parameter: cv::Mat & image
	// Parameter: cv::Point & pos
	// Parameter: std::string str
	//************************************
	static void putString(cv::Mat &image, cv::Point &pos, std::string str);

	static cv::Rect rectROI;
	static Line boundInit[3];
	static int boundIndex;

	//求交界线的限定范围---40公分
	 static int BOUND_LEFT_LOW;
	 static int BOUND_LEFT_HIGH;
	 static int BOUND_RIGHT_LOW;
	 static int BOUND_RIGHT_HIGH;

	//求交界位置或打标记在X方向上的范围-40公分位置
	 static int MARK_LEFT;
	 static int MARK_RIGHT;
	//找交界位置在X方向上的中间位置
	 static int MARK_MID;

	//打标记在y方向上的范围---尾部摄像头
	 static int MARK_LOW;
	 static int MARK_HIGH;
};

