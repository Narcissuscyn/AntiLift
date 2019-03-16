#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
class LineDetect
{
public:
	LineDetect(const std::string&filePath, int houghFlag = 0);
	LineDetect(cv::Mat &frame);

	~LineDetect();
	std::vector<cv::Vec2f> getLines2f() const { return lines2f; }
	void setLines2f(std::vector<cv::Vec2f> val) { lines2f = val; }
	std::vector<cv::Vec4i> getLines4i() const { return lines4i; }
	void setLines4i(std::vector<cv::Vec4i> val) { lines4i = val; }

	
	std::vector<cv::Vec4f> getLinesFLD() const { return lines_fld; }
	void setLinesFLD(std::vector<cv::Vec4f> val) { lines_fld = val; }

	cv::Mat getPontsLsd() const { return pontsLsd; }
	void setPontsLsd(cv::Mat val) { pontsLsd = val; }
private:
	std::vector<cv::Vec2f> lines2f;
	std::vector<cv::Vec4i> lines4i;
	cv::Mat pontsLsd;
	std::vector<cv::Vec4f> lines_fld;


	//************************************
	// Method:    LoadImage
	// FullName:  HoughTest::LoadImage
	// Access:    private 
	// Returns:   void
	// Qualifier:
	// Parameter: const std::string   filePath 文件名
	// Parameter: int flag   选择houghlines或者houghLinesP
	//************************************
	void lineDetectHoughLine(cv::Mat &frame, int flag);

	void  LineDetectLSD(cv::Mat &srcImage);

};

