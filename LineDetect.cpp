#include "LineDetect.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv.hpp>
#include "lsd.h"
using namespace al;
using namespace cv;
using namespace std;



LineDetect::LineDetect(const std::string&filePath, int houghFlag)
{
	//Mat src, dst, color_dst;
	Mat srcImage = imread(filePath);  //打开的原始图像
	if (srcImage.empty())
	{
		cout << "File open error!" << endl;
		return;
	}
	else
	{
		cout << "open file:" << filePath << " ok!" << endl;
		cv::imshow("src:", srcImage);
	}
	//LoadImage(srcImage, houghFlag);
	LineDetectLSD(srcImage);

}

LineDetect::LineDetect(cv::Mat &frame)
{
	//LoadImage(frame, houghFlag);
	LineDetectLSD(frame);
//	lineDetectHoughLine(frame, true);
	//lineDetectHoughLine(frame, true);
}


LineDetect::~LineDetect()
{
}


void LineDetect::lineDetectHoughLine(Mat &srcImage,int flag)
{
	
	Mat midImage, dstImage;//临时变量和目标图的定义
	midImage = srcImage;
	dstImage = srcImage;
	//Canny(midImage, midImage, 120, 240, 3);//进行一此canny边缘检测
	//imshow("canny", midImage);
	//cvtColor(midImage, dstImage, CV_GRAY2BGR);//转化边缘检测后的图为灰度图


#if 0
	HoughLines(midImage, lines2f, 1, CV_PI / 180, 290, 0, 0);

	for (size_t i = 0; i < lines2f.size(); i++)
	{
		float rho = lines2f[i][0];
		float theta = lines2f[i][1];
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		Point pt1(cvRound(x0 + 1000 * (-b)),
			cvRound(y0 + 1000 * (a)));
		Point pt2(cvRound(x0 - 1000 * (-b)),
			cvRound(y0 - 1000 * (a)));
		line(srcImage, pt1, pt2, Scalar(0, 0, 255), 3, 8);
	}
#else
	/*
	Parameters:
	image – 8位单通道图像
	lines – Vec4i类型的向量：直线的两个端点（ (x_1, y_1, x_2, y_2) ）
	rho – 累加器的距离分辨率（以像素为单位）
	theta – 累加器的角度分辨率（以弧度为单位）
	threshold –累加器阈值参数，大于这个阈值参数的直线才会被保留
	minLineLength – 直线的最短长度 
	maxLineGap –将两个线段当成一条直线的最大距离（这个距离之内的相邻两条线段在一条直线上将被合并）
	*/
	HoughLinesP(midImage, lines4i,3, CV_PI /90, 200, 250, 200);
	//HoughLinesP(midImage, lines4i, 2.5, CV_PI / 45, 250, 250, 150);

	for (size_t i = 0; i < lines4i.size(); i++)
	{
		circle(dstImage, Point(lines4i[i][0], lines4i[i][1]), 5, Scalar(255, 0, 0), 3, 8);

		circle(dstImage, Point(lines4i[i][2], lines4i[i][3]), 5, Scalar(0, 255, 0), 4, 8);
		line(dstImage, Point(lines4i[i][0], lines4i[i][1]),Point(lines4i[i][2], lines4i[i][3]), Scalar(0, 0, 255), 1, 8);
	}
#endif
	
	//cout <<"WaitKey（10）的返回值："<< waitKey(10) << endl;
	//cv::imshow("lineTest", dstImage);
	return ;
}


void LineDetect::LineDetectLSD(Mat &srcImage)
{
	
	/*int scale = 1;
	int delta = 0;
	GaussianBlur(image, image, Size(3, 3), 0, 0, BORDER_DEFAULT);
	int ddepth = CV_16S;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	Sobel(image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	Sobel(image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, image);*/

	/*int blockSize =21;
	int constValue = 7;
	adaptiveThreshold(image, image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	*/
	
#if 0
	Canny(image, image, 50, 200, 3); // Apply canny edge
#endif
	// Create and LSD detector with standard or no refinement.
#if 1
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_ADV, 0.8, 0.6, 2.0, 25.5, 0, 0.7, 1024);
	pontsLsd.release();
	ls->detect(srcImage, pontsLsd);
	Mat drawnLines(srcImage);
	ls->drawSegments(drawnLines, getPontsLsd());
	/*cv::imshow("2.直线检测结果", drawnLines);
	cv::imwrite("2直线检测结果.jpg", drawnLines);
*/
#else
	int length_threshold = 10;
	float distance_threshold = 1.41421356f;
	double canny_th1 = 50.0;
	double canny_th2 = 50.0;
	int canny_aperture_size = 3;
	bool do_merge = false;
	Ptr<FastLineDetector> fld = createFastLineDetector(length_threshold,
		distance_threshold, canny_th1, canny_th2, canny_aperture_size,
		do_merge);
	lines_fld.clear();
	fld->detect(image, lines_fld);
	Mat line_image_fld(image);
	fld->drawSegments(line_image_fld, getLinesFLD());
	imshow("FLD result", line_image_fld);

	//Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
#endif

}

