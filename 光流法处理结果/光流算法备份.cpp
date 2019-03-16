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
	/*TermCriteriaģ���࣬ȡ����֮ǰ��CvTermCriteria��
	���������Ϊ�����㷨����ֹ�����ģ�������ڲο��ֲ���
	���ܵĺܼ򵥣��Ҳ���Щ���ϣ��������һ�¡����������
	Ҫ3��������һ�������ͣ��ڶ�������Ϊ����������������
	��һ�����ض�����ֵ��������CV_TERMCRIT_ITER��
	CV_TERMCRIT_EPS��CV_TERMCRIT_ITER+CV_TERMCRIT_EPS��
	�ֱ�����ŵ�����ֹ����Ϊ�ﵽ������������ֹ����������
	ֵ��ֹ���������߶���Ϊ������ֹ���������ϵĺ��Ӧ��c++��
	�汾�ֱ�ΪTermCriteria::COUNT��TermCriteria::EPS��
	�����COUNTҲ����д��MAX_ITER��
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
		//����һ֡ͼƬ
		cap >> frame;
		if (frame.empty())
			break;
		frame.copyTo(image);

		//ת���ɻҶ�ͼ
		cvtColor(image, gray, COLOR_BGR2GRAY);

// 
// 		if (nightMode)
// 			image = Scalar::all(0);
	/*//��ʼ���ǵ���--------------����ط���Ҫ�Ľ��������ĸ���ͬ���������
	���ò�ͬ�ı��λ�ã�����ÿ�����������,��ǵ�λ�û������䣩��
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

			//copyһ�ݻҶ�ͼ��
			if (prevGray.empty())
				gray.copyTo(prevGray);

			//��������
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
