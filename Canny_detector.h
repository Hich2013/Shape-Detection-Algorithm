#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>


class Canny_detector
{

public:

	void  hys_threshold(float upperDir, float lowerDir, cv::Mat mat1, cv::Mat mat2, cv::Mat mat3, int lowT, float p1,
		float p2, float p3, float p4, float p5, float p6, float p7, float p8, bool is_chg);
	
	cv::Mat MyCannyDetector(cv::Mat sourceImg, int upperThreshold, int lowerThreshold, float size);
	
	static double angle_cosine(cv::Point pt1, cv::Point pt2, cv::Point pt0);

	void labelShape(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

	// Compute Euclidean distance between two points 
	inline float euc_dist(cv::Point& p, cv::Point& q)
	{
		cv::Point diff = p - q;
		return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
	}

	void CannyThreshold(int, void*);
	
};
