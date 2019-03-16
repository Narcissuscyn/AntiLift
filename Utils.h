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

	//�󽻽��ߵ��޶���Χ---40����
	 static int BOUND_LEFT_LOW;
	 static int BOUND_LEFT_HIGH;
	 static int BOUND_RIGHT_LOW;
	 static int BOUND_RIGHT_HIGH;

	//�󽻽�λ�û������X�����ϵķ�Χ-40����λ��
	 static int MARK_LEFT;
	 static int MARK_RIGHT;
	//�ҽ���λ����X�����ϵ��м�λ��
	 static int MARK_MID;

	//������y�����ϵķ�Χ---β������ͷ
	 static int MARK_LOW;
	 static int MARK_HIGH;
};

