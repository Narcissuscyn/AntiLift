#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc.hpp"
//#include "videoio.hpp"
//#include "opencv2/videoio.hpp"
//#include <highgui.hpp>
#include <string>
#include <vector>


class OpticalFlow
{
public:
	//************************************
	// Method:    OpticalFlow
	// FullName:  OpticalFlow::OpticalFlow
	// Access:    public 
	// Returns:   
	// Qualifier:
	// Parameter: const std::string filePath
	//************************************
	OpticalFlow(const std::string filePath);

	//************************************
	// Method:    getCap
	// FullName:  OpticalFlow::getCap
	// Access:    public 
	// Returns:   cv::VideoCapture
	// Qualifier: const
	//************************************
	cv::VideoCapture getCap() const { return cap; }
	void setCap(cv::VideoCapture val) { cap = val; }


	//************************************
	// Method:    calOpticalFlow
	// FullName:  OpticalFlow::calOpticalFlow
	// Access:    public 
	// Returns:   std::vector<cv::Point2f>
	// Qualifier:
	// Parameter: std::vector<cv::Point2f> & initialPoints
	// Parameter: std::vector<int> & pointFlag
	// Parameter: cv::Mat & lastFrame
	//************************************
	std::vector<cv::Point2f> calOpticalFlow(std::vector<cv::Point2f> &initialPoints, std::vector<int>&pointFlag, cv::Mat &lastFrame);
private:
	cv::VideoCapture cap;

};

