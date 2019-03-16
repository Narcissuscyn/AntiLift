#pragma once
#ifdef MATHFUNCSDLL_EXPORTS
#define ANTILIFT5_API __declspec(dllexport) 
#else
#define ANTILIFT5_API __declspec(dllimport) 
#endif
#include "InitMarks.h"
#include"Utils.h"
#include <opencv.hpp>
namespace antilift
{
	class  ANTILIFT5_API AntiLiftClass
	{
	public:
		//************************************
		// Method:    AntiLiftClass
		// FullName:  AntiLiftClass::AntiLiftClass
		// Access:    public 
		// Returns:   no return
		// Parameter: int camPos----------------摄像机位置参数：1-尾部摄像头或2-头部摄像头
		//
		// Parameter: int boxSize----------------箱子尺寸参数：40 或者 20
		//************************************
		AntiLiftClass();
		//************************************
		// Method:    AntiLiftClass
		// FullName:  AntiLiftClass::AntiLiftClass
		// Access:    public 
		// Returns:   no return
		// Parameter: int camPos----------------摄像机位置参数：1-尾部摄像头或2-头部摄像头
		//
		// Parameter: int boxSize----------------箱子尺寸参数：40 或者 20
		//************************************
		void initParams(int boxSize, int camPos, cv::Rect r, Line boundInit[3]);

		//************************************
		// Method:    antiLift
		// FullName:  AntiLiftClass::antiLift
		// Access:    public 
		// Returns:   void
		// Qualifier:
		// Parameter: cv::Mat frame
		//************************************
		void antiLift(cv::Mat frame);

		void initProc();
		void trackProc();
		void judgeProc();
		~AntiLiftClass();
		bool getJudgeSignal() const;
		//************************************
		// Method:    setJudgeSignal
		// FullName:  AntiLiftClass::setJudgeSignal
		// Access:    public 
		// Returns:   void
		// Qualifier:
		// Parameter: bool val
		//************************************
		void setJudgeSignal(bool val);
		bool getIsFirsFram() const;
		void setIsFirsFram(bool val);
		int getCamPos() const;
		void setCamPos(int val);
		int getPosFlag() const;
		void setBoxSizeFlag(int val);
		bool getInitSignal() const;
		//************************************
		// Method:    setInitSignal
		// FullName:  AntiLiftClass::setInitSignal
		// Access:    public 
		// Returns:   void
		// Qualifier:
		// Parameter: bool val
		//************************************
		void setInitSignal(bool val);
		bool getIsFinalSignal() const;
		//************************************
		// Method:    setIsFinalSignal
		// FullName:  AntiLiftClass::setIsFinalSignal
		// Access:    public 
		// Returns:   void
		// Qualifier:
		// Parameter: bool val
		//************************************
		void setIsFinalSignal(bool val);
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
		int getFinalResult() const;
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
		void setFinalResult(int val);

		void initProgram(cv::Mat frame);
	private:
		bool judgeSignal = false;
		bool initSignal = false;
		bool isFirsFram = true;
		bool isFinalSignal = false;
		/*
		the final result：
		1-truck isn't lifted
		0-without judging
		-1-truck is lifted
		*/
		int finalResult = 0;
		/*
		if continuous 10 frames alert,give the final rasult
		*/
		int alertSuccFram = 0;
		/*
		if continuous 5 frames fail to alert,alertSuccFrame will be set to 0;
		*/
		int alertFailFram = 0;
		/*
		points thats been marked in the first frame
		*/
		cv::Mat initialPoints;
		/*
		each marked point has a flag;
		-2: points under the boundary
		-1:points under the boundary needed to be clear
		1:  points over the boundary needed to be clear
		2: points over the boundary
		*/
		cv::Mat pointsFlag;
		/*
		the first frame is used to initialize the data
		*/
		cv::Mat firstFrame;
		/*
		the object of InitMarks
		*/
		InitMarks *initMarks;
		/*
		the Camera Position:
		1-in the end of the car
		2-in the head of the car
		*/
		int camPos;
		/*
		the box position flag:
		40-in the 40 feets position
		20-in the 20 feets position
		*/
		int posFlag;
		cv::Mat pointsPre;
		cv::Mat pointsNex;
		cv::Mat nexGray;
		cv::Mat preGray;
		cv::Mat curFrame;

	};
}