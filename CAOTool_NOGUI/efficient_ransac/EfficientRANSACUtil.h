#pragma once

#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace efficient_ransac {

	float angle_difference(float x, float xi);
	float regularize_angle_PI(float x);
	float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segment_only);

}
