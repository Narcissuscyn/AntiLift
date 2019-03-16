#include "LeastSquare.h"
using namespace std;
using namespace cv;
LeastSquare::LeastSquare(const cv::Mat& pVec)
{
	double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
	for (int i = 0; i < pVec.rows; ++i)
	{
		t1 += pVec.at<cv::Point2f>(i).x*pVec.at<cv::Point2f>(i).x;
		t2 += pVec.at<cv::Point2f>(i).x;
		t3 += pVec.at<cv::Point2f>(i).x * pVec.at<cv::Point2f>(i).y;
		t4 += pVec.at<cv::Point2f>(i).y;
	}
	setA((t3*pVec.rows - t2*t4) / (t1*pVec.rows - t2*t2));  // 求得β1 
	setB((t1*t4 - t2*t3) / (t1*pVec.rows - t2*t2));        // 求得β2
}
LeastSquare::~LeastSquare()
{
}
