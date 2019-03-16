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
	double rate = cap.get(CV_CAP_PROP_FPS); // ��ȡ
	long nFrame = static_cast<long>(cap.get(CV_CAP_PROP_FRAME_COUNT)); // ��ȡ��֡��

	cout << "��Ƶ��Ϣ:֡��=" << rate << "  ��֡��=" << nFrame << endl;

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
		//ת���ɻҶ�ͼ
		cvtColor(image, NexGray, COLOR_BGR2GRAY);

		if (!pointsPre.empty())
		{
			double duration;
			duration = static_cast<double>(cv::getTickCount());
		
			vector<uchar> status;
			vector<float> err;

			//copyһ�ݻҶ�ͼ��
			if (preGray.empty())
				NexGray.copyTo(preGray);

			//��������--�о���Щ����
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
			//���ڸ��ٶ�ʧ�ĵ�Ĵ���
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
		//	cout << "�˶����ٵ�����ʱ�䣺" << duration << endl;

		}

		//�źű���������ʼ�жϽ����
		//imshow("LKoutput:", image);

		VideoWriter writer;
		writer.open("result.avi", CV_FOURCC('M', 'G', 'P', 'G'), cap.get(CV_CAP_PROP_FPS), image.size(),1);
		writer << image;


		char c = (char)waitKey(10);
		if (c == 27)
			break;

		std::swap(pointsNex, pointsPre);
		cv::swap(preGray, NexGray);//��һ֡������ɣ���gray������prevgray,��һ��ѭ����gray������µ�֡

		

		cap >> frame;


		

	}
	//imshow("LastFrame:", image);
	//imwrite("lastFrame.jpg", image);
	lastFrame = image;
	cout << "��Ƶ���������" << endl;
	return pointsPre;
}
