#include"Utils.h"
Utils::Utils()
{

}
Utils::~Utils()
{

}

 cv::Mat Utils::putWaterMark(cv::Mat &image, cv::Mat &logo, cv::Point pos)
{
	cv::Mat imageROI;
	imageROI = image(cv::Rect(pos.x, pos.y, logo.cols, logo.rows));
	logo.copyTo(imageROI);
	return image;
}

 void Utils::putString(cv::Mat &image, cv::Point &pos, std::string str)
{
	//cout << str << endl;
	cv::putText(image, str, pos, CV_FONT_HERSHEY_SIMPLEX, 0.8,
		cv::Scalar(0, 0, 0), 2);
	pos.y += 25;
}

 cv::Rect Utils::rectROI;
 Line Utils::boundInit[3];
 int Utils::boundIndex;

 //求交界线的限定范围---40公分
 int Utils::BOUND_LEFT_LOW;
 int Utils::BOUND_LEFT_HIGH;
 int Utils::BOUND_RIGHT_LOW;
 int Utils::BOUND_RIGHT_HIGH;

 //求交界位置或打标记在X方向上的范围-40公分位置
 int Utils::MARK_LEFT;
 int Utils::MARK_RIGHT;
 //找交界位置在X方向上的中间位置
 int Utils::MARK_MID;

 //打标记在y方向上的范围---尾部摄像头
 int Utils::MARK_LOW;
 int Utils::MARK_HIGH;