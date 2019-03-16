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

 //�󽻽��ߵ��޶���Χ---40����
 int Utils::BOUND_LEFT_LOW;
 int Utils::BOUND_LEFT_HIGH;
 int Utils::BOUND_RIGHT_LOW;
 int Utils::BOUND_RIGHT_HIGH;

 //�󽻽�λ�û������X�����ϵķ�Χ-40����λ��
 int Utils::MARK_LEFT;
 int Utils::MARK_RIGHT;
 //�ҽ���λ����X�����ϵ��м�λ��
 int Utils::MARK_MID;

 //������y�����ϵķ�Χ---β������ͷ
 int Utils::MARK_LOW;
 int Utils::MARK_HIGH;