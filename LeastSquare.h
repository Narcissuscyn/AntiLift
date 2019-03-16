#pragma once
#include <vector>
#include <iostream>
#include <opencv.hpp>
class LeastSquare
{
public:
	LeastSquare(const cv::Mat & pMat);
	~LeastSquare();
	double getY(const double x) const
	{
		return getA()*x + getB();
	}

	void print() const
	{
		std::cout << "y = " << getA() << "x + " << getB() << "\n";
	}
	double getA() const { return a; }
	void setA(double val) { a = val; }
	double getB() const { return b; }
	void setB(double val) { b = val; }
private:
	double a, b;
};

