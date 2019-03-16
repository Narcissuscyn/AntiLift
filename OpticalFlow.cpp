#include "OpticalFlow.h"
#include "opencv2/video/tracking.hpp"
using namespace cv;
using namespace std;
#include <iostream>
const int MAX_CORNERS = 10000;
bool addRemovePt = false;
Point2f point;

OpticalFlow::OpticalFlow(const std::string filePath)
{
	cap.open(filePath);	
	double rate = cap.get(CV_CAP_PROP_FPS); // 获取
	long nFrame = static_cast<long>(cap.get(CV_CAP_PROP_FRAME_COUNT)); // 获取总帧数

	cout << "视频信息:帧率=" << rate << "  总帧数=" << nFrame << endl;

}
std::vector<cv::Point2f> OpticalFlow::calOpticalFlow( std::vector<cv::Point2f> &initialPoints, std::vector<int>&pointFlag, cv::Mat &lastFrame)
{
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size   winSize(31, 31);

	std::vector<cv::Point2f> pointsPre;
	pointsPre.clear();
	std::vector<cv::Point2f>  pointsNex= initialPoints;
	Mat NexGray, preGray, image, frame;
	cap >> frame;
	while (!frame.empty())
	{
		frame.copyTo(image);
		//转换成灰度图
		cvtColor(image, NexGray, COLOR_BGR2GRAY);

		if (!pointsPre.empty())
		{
			double duration;
			duration = static_cast<double>(cv::getTickCount());
		
			vector<uchar> status;
			vector<float> err;

			//copy一份灰度图像
			if (preGray.empty())
				NexGray.copyTo(preGray);

			//光流计算--研究这些参数
			/*
			C++: void calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts,
			InputOutputArray nextPts, OutputArray status, OutputArray err, Size winSize=Size(15,15), 
			int maxLevel=3, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
		  double derivLambda=0.5, int flags=0 )
			*/
			calcOpticalFlowPyrLK(preGray, NexGray, pointsPre, pointsNex, status, err, winSize,
				3, termcrit, 0, 0.00001);

			int i=0, k=0;
			for (i = k = 0; i < pointsNex.size(); i++)
			{
			//对于跟踪丢失的点的处理
				if (!status[i])
				{
				//	initialPoints.erase;
					continue;
				}

				pointsNex[k] = pointsNex[i];
				initialPoints[k] = initialPoints[i];
				pointFlag[k] = pointFlag[i];
				k++;
				circle(image, pointsNex[i], 4, Scalar(0, 0, 255), -1, 8);
			}
			pointsNex.resize(k);
			initialPoints.resize(k);
			pointFlag.resize(k);
		
			duration = static_cast<double>(cv::getTickCount()) - duration;
			duration /= cv::getTickFrequency(); // the elapsed time in ms
		//	cout << "运动跟踪的运行时间：" << duration << endl;

		}

		//信号被激发，开始判断结果：
		//imshow("LKoutput:", image);

		VideoWriter writer;
		writer.open("result.avi", CV_FOURCC('M', 'G', 'P', 'G'), cap.get(CV_CAP_PROP_FPS), image.size(),1);
		writer << image;


		char c = (char)waitKey(10);
		if (c == 27)
			break;

		std::swap(pointsNex, pointsPre);
		cv::swap(preGray, NexGray);//这一帧处理完成，将gray交换给prevgray,下一次循环，gray将获得新的帧

		

		cap >> frame;


		

	}
	//imshow("LastFrame:", image);
	//imwrite("lastFrame.jpg", image);
	lastFrame = image;
	cout << "视频输出结束！" << endl;
	return pointsPre;
}
