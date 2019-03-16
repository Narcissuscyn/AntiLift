#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

const int MAX_CORNERS = 500;

vector<Point2f>initialPos;
vector<Point2f>endPos;


static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
		"Using OpenCV version " << CV_VERSION << endl;
	cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tr - auto-initialize tracking\n"
		"\tc - delete all the points\n"
		"\tn - switch the \"night\" mode on/off\n"
		"To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		point = Point2f((float)x, (float)y);
		addRemovePt = true;
	}
}

int main(int argc, char** argv)
{
	help();


	VideoCapture cap;
	/*TermCriteria模板类，取代了之前的CvTermCriteria，
	这个类是作为迭代算法的终止条件的，这个类在参考手册里
	介绍的很简单，我查了些资料，这里介绍一下。该类变量需
	要3个参数，一个是类型，第二个参数为迭代的最大次数，最
	后一个是特定的阈值。类型有CV_TERMCRIT_ITER、
	CV_TERMCRIT_EPS、CV_TERMCRIT_ITER+CV_TERMCRIT_EPS，
	分别代表着迭代终止条件为达到最大迭代次数终止，迭代到阈
	值终止，或者两者都作为迭代终止条件。以上的宏对应的c++的
	版本分别为TermCriteria::COUNT、TermCriteria::EPS，
	这里的COUNT也可以写成MAX_ITER。
	*/
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	//const int MAX_COUNT = 500;
	bool needToInit = false;
	bool nightMode = false;

// 	cv::CommandLineParser parser(argc, argv, "{@input||}{help h||}");
// 	string input = parser.get<string>("@input");
// 	if (parser.has("help"))
// 	{
// 		help();
// 		return 0;
// 	}

	cap.open("test3.mp4");
	namedWindow("LK Demo", CV_WINDOW_NORMAL);
	setMouseCallback("LK Demo", onMouse, 0);

	Mat gray, prevGray, image, frame;

	vector<Point2f> points[2];

	for (;;)
	{
		//读入一帧图片
		cap >> frame;
		if (frame.empty())
			break;
		frame.copyTo(image);

		//转换成灰度图
		cvtColor(image, gray, COLOR_BGR2GRAY);

// 
// 		if (nightMode)
// 			image = Scalar::all(0);
	/*//初始化角点检测--------------这个地方需要改进，根据四个不同的摄像机，
	采用不同的标记位置（对于每个摄像机而言,标记的位置基本不变）。
	*/

		if (needToInit)
		{
			// automatic initialization
			goodFeaturesToTrack(gray, points[1], MAX_CORNERS, 0.01, 5.0, Mat(), 3, 0, 0.04);
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
			addRemovePt = false;
		}
		else if (!points[0].empty())
		{
			vector<uchar> status;
			vector<float> err;

			//copy一份灰度图像
			if (prevGray.empty())
				gray.copyTo(prevGray);

			//光流计算
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);

			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++)
			{
				if (addRemovePt)
				{
					if (norm(point - points[1][i]) <= 5)
					{
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 4, Scalar(0, 0, 255), -1, 8);
			}
			points[1].resize(k);
		}

		if (addRemovePt && points[1].size() < (size_t)MAX_CORNERS)
		{
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}

		needToInit = false;
		imshow("LK Demo", image);

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'r':
			needToInit = true;
			break;
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		case 'n':
			nightMode = !nightMode;
			break;
		}

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
}
