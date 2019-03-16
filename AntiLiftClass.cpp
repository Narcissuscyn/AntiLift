#include "AntiLiftClass.h"
#include "LineDetect.h"
#include "OpticalFlow.h"
#include "LeastSquare.h"
#include "CalResult.h"
#include "PreProcessPoints.h"
#include <fstream>

using namespace std;
using namespace cv;
using namespace antilift;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size winSize(31, 31);

	bool isFirst = true;

	AntiLiftClass::AntiLiftClass()
	{
		
	}
	void AntiLiftClass::initParams(int boxSize,int camPos,cv::Rect r, Line line[3])
	{

		setBoxSizeFlag(boxSize);
		setCamPos(camPos);
		Utils::rectROI = r;
		for (int i = 0; i < 3; i++)
		{
			Utils::boundInit[i] = line[i];
		}
		isFirsFram = true;
		initMarks = new InitMarks(getCamPos(), getPosFlag());
	}
	AntiLiftClass::~AntiLiftClass()
	{
		delete initMarks;
		initialPoints.release();
		pointsPre.release();
		pointsNex.release();
		nexGray.release();
		preGray.release();
		curFrame.release();
		firstFrame.release();
		pointsFlag.release();
	}
	void AntiLiftClass::antiLift(cv::Mat frame)
	{
		//改变图像亮度与对比度
		frame.convertTo(frame, 0, 1.0, 80);
		if (getIsFinalSignal())
		{
			setFinalResult(1);
			return;
		}
		if (frame.empty())
		{
			std::cout << "thr frame is empty!" << endl;
			return;
		}
		if (getInitSignal())
		{
			if (getIsFirsFram())
			{
				frame.copyTo(firstFrame);
				//对图片进行graphcut分割
				/*GraphCut graphCut(firstFrame);
				firstFrame=graphCut.getResImg();*/
				//imwrite("1.jpg",firstFrame);
				initProc();
			}
			else
			{
				//4.use Opticalflow to track the initialized points 
				frame.copyTo(curFrame);
				trackProc();
				//5.if the judge signal is aroused ,then preprocessing the tracked points and get the results of the current frame
				judgeProc();
				//	cv::imwrite("zhuanli7.jpg", curFrame);
					//cv::imshow("LKoutput:", curFrame);
			}
		}

	}
	void AntiLiftClass::initProc()
	{
		cv::Mat pointsLsd;
		Rect rect;
		rect.x = Utils::rectROI.x;
		rect.y = Utils::rectROI.y;
		rect.width = Utils::rectROI.width;
		rect.height = Utils::rectROI.height;
		/*cout << "rect.x=" << Utils::rectROI.x << endl;
		cout << "rect.y=" << Utils::rectROI.y << endl;
		cout << "rect.width=" << Utils::rectROI.width << endl;
		cout << "rect.height=" << Utils::rectROI.height << endl;*/

		//1.line detect
		Mat image = firstFrame;
		cvtColor(image, image, CV_BGR2GRAY);
		equalizeHist(image, image);
		Mat imgToLsd(image.size(), CV_8UC1);
		imgToLsd = cv::Scalar::all(0);
		Mat imgRoi = imgToLsd(rect);//imgRoi相当于是imgToLsd的rect区域的引用
		image(rect).copyTo(imgRoi);

		//imshow ("1.直线检测图像", imgToLsd);
		/*imwrite("1直线检测图像.jpg", imgToLsd);*/
		LineDetect lineDetect(imgToLsd);//采用LSD直线检测算法

		pointsLsd = lineDetect.getPontsLsd();
		Mat pointsToLS = initMarks->getLsdPointsToLS(pointsLsd);

		Mat framGetBund = firstFrame.clone();
		for (int i = 0; i < pointsToLS.rows; i++)
		{
			circle(framGetBund, Point(pointsToLS.at<Point2f>(i).x, pointsToLS.at<Point2f>(i).y), 4, Scalar(255, 255, 255), 5, 8);
		}
		/*cv::imshow("3.拟合直线的点", framGetBund);
		cv::imwrite("3拟合直线的点.jpg", framGetBund);
	*/
	//2.use LeastSquare to get the boundary line;
		LeastSquare ls(pointsToLS);
		ls.print();
		Point2i signPosBeg(0, 0);
		Point2i signPosEnd(0, 0);
		signPosBeg.x = 50;
		signPosBeg.y = ls.getY(signPosBeg.x);
		signPosEnd.x = 1300;
		signPosEnd.y = ls.getY(signPosEnd.x);
		line(framGetBund, signPosBeg, signPosEnd, Scalar(255, 0, 0), 2, 8);
		/*cv::imshow("4.交界直线", framGetBund);
		cv::imwrite("4交界直线.jpg", framGetBund);*/

		//3.initialize the marked points 
		initMarks->markFirstFrame(ls, initialPoints, pointsFlag, firstFrame);
		initialPoints.copyTo(pointsNex);
		setIsFirsFram(false);
	}

	void AntiLiftClass::trackProc()
	{
		cvtColor(curFrame, nexGray, COLOR_BGR2GRAY);
		//cv::imshow("nexGray:", nexGray);
		//直方图均衡化
		equalizeHist(nexGray, nexGray);
		//cv::imshow("nexGray equalizeHist:", nexGray);

		if (!pointsPre.empty() && !nexGray.empty() && !preGray.empty())
		{
			cv::Mat err;
			cv::Mat status;
			//use Opticalflow
			calcOpticalFlowPyrLK(preGray, nexGray, pointsPre, pointsNex, status, err, winSize,
				5, termcrit, 0, 0.00005);
			//use RANSAC
			Mat H12;   //变换矩阵
			Mat points1, points2;
			pointsPre.copyTo(points1);
			pointsNex.copyTo(points2);
			int ransacReprojThreshold = 6;  //拒绝阈值
			H12 = findHomography(Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold);
			Mat points1t;
			perspectiveTransform(Mat(points1), points1t, H12);
			int k = 0;
			for (size_t i = 0; i < points1.rows; i++)  //保存‘内点’
			{
				if (norm(points2.at<Point2f>(i) - points1t.at<Point2f>((int)i, 0)) <= ransacReprojThreshold) //给内点做标记
				{
					pointsPre.at<Point2f>(k) = pointsPre.at<Point2f>(i);
					pointsNex.at<Point2f>(k) = pointsNex.at<Point2f>(i);
					initialPoints.at<Point2f>(k) = initialPoints.at<Point2f>(i);
					pointsFlag.at<int>(k) = pointsFlag.at<int>(i);
					k++;
					circle(curFrame, Point(pointsNex.at<Point2f>(i)), 4, Scalar(255, 0, 0), 1, 8);
				}
			}
			pointsPre.resize(k);
			pointsNex.resize(k);
			initialPoints.resize(k);
			pointsFlag.resize(k);
		}

	}
	void AntiLiftClass::judgeProc()
	{
		if (getJudgeSignal())
		{
			cv::Mat initialPoints1;
			cv::Mat endPoints1;
			cv::Mat pointsFlag1;

			initialPoints.copyTo(initialPoints1);
			pointsNex.copyTo(endPoints1);
			pointsFlag.copyTo(pointsFlag1);
			//PreProcessPoints::clearPoints(initialPoints1, endPoints1, pointsFlag1);
			Mat lineUp1, lineDown1, initDown1, initUp1;

			Mat whiteFrame(firstFrame.size(), CV_32FC3);
			PreProcessPoints::distinguishPoints(initialPoints1, endPoints1, pointsFlag1, lineUp1, lineDown1, initDown1, initUp1, whiteFrame);

			CalResult calResult(lineDown1, initDown1, lineUp1, initUp1, initMarks->getInitPoinDownNumb(), initMarks->getInitPoinUpNumb());
			if (camPos == 3)
			{
				calResult.setDxDanger(20);
				calResult.setDyDanger(120);
				calResult.getResultSide(curFrame);
			}
			else
			{
				calResult.setDxDanger(8);
				calResult.setDyDanger(12);
				calResult.getResults(curFrame);
			}

			if (calResult.getIsCurrFramAlert())
			{
				alertSuccFram++;
				alertFailFram = 0;
			}
			else
			{
				if (alertSuccFram > 0)
				{
					//if 0<alertSuccFram<10 and current frame failed to arouse an alert,let alertSuccFram=0
					if (alertFailFram < 3)
					{
						alertFailFram++;
					}
					else
					{
						alertFailFram = 0;
						alertSuccFram = 0;
					}
				}
			}
			//	imshow("7.当前帧判断图片结果", curFrame);
			if (isFirst)
			{
				//	imwrite("7第一帧判断图片结果.jpg",curFrame);
				isFirst = false;
			}
		}

		//if ten alerts are aroused continuously,we give the final result
		if (alertSuccFram >= 5)
		{
			setFinalResult(-1);
			setJudgeSignal(false);
			setInitSignal(false);
			return;
		}
		//switch the current data to “pointsPre” and “preGray”，so as to caculate next frame
		pointsNex.copyTo(pointsPre);
		nexGray.copyTo(preGray);
		nexGray.release();
		pointsNex.release();
		return;
	}


	bool AntiLiftClass::getJudgeSignal() const
	{
		return judgeSignal;
	}
	//************************************
	// Method:    setJudgeSignal
	// FullName:  AntiLiftClass::setJudgeSignal
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: bool val
	//************************************
	void AntiLiftClass::setJudgeSignal(bool val)
	{
		judgeSignal = val;
	}
	bool AntiLiftClass::getIsFirsFram() const { return isFirsFram; }
	void AntiLiftClass::setIsFirsFram(bool val) { isFirsFram = val; }
	int AntiLiftClass::getCamPos() const { return camPos; }
	void AntiLiftClass::setCamPos(int val) { camPos = val; }
	int AntiLiftClass::getPosFlag() const { return posFlag; }
	void AntiLiftClass::setBoxSizeFlag(int val) { posFlag = val; }
	bool AntiLiftClass::getInitSignal() const { return initSignal; }
	//************************************
	// Method:    setInitSignal
	// FullName:  AntiLiftClass::setInitSignal
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: bool val
	//************************************
	void AntiLiftClass::setInitSignal(bool val) { initSignal = val; }
	bool AntiLiftClass::getIsFinalSignal() const { return isFinalSignal; }
	//************************************
	// Method:    setIsFinalSignal
	// FullName:  AntiLiftClass::setIsFinalSignal
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: bool val
	//************************************
	void AntiLiftClass::setIsFinalSignal(bool val) { isFinalSignal = val; }
	/*
	the final result：
	1-truck isn't lifted
	0-without judging
	-1-truck is lifted
	*/
	//************************************
	// Method:    getFinalResult
	// FullName:  AntiLiftClass::getFinalResult
	// Access:    public 
	// Returns:   int
	// Qualifier: const
	//************************************
	int AntiLiftClass::getFinalResult() const { return finalResult; }
	/*
	the final result：
	1-truck isn't lifted
	0-without judging
	-1-truck is lifted
	*/
	//************************************
	// Method:    setFinalResult
	// FullName:  AntiLiftClass::setFinalResult
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: int val
	//************************************
	void AntiLiftClass::setFinalResult(int val) { finalResult = val; }

	void AntiLiftClass::initProgram(cv::Mat frame)
	{
		/*
		在main函数中调用
		antiLift.initProgram(frame);
		cv::waitKey(0);
		cout << "initial ok!" << endl;*/
		/*InitProgram initProfram;
		initProfram.init(frame);*/
	}
