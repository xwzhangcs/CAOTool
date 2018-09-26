#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dlib/optimization.h>
#include "../util/ContourUtils.h"

class SymmetryLineDetector {

	typedef dlib::matrix<double, 0, 1> column_vector;

	class BFGSSolver {
	private:
		std::vector<cv::Point2f> target_polygon;

	public:
		BFGSSolver(const std::vector<cv::Point2f>& target_polygon) {
			this->target_polygon = target_polygon;


		}

		double operator() (const column_vector& arg) const {
			std::vector<cv::Point2f> polygon;
			cv::Point2f a = cv::Point2f(arg(0), arg(1));
			cv::Point2f b = cv::Point2f(arg(0) + 0.02 * cos(arg(2)), arg(1) + 0.02 * sin(arg(2)));
			for (int i = 0; i < target_polygon.size(); i++) {
				polygon.push_back(util::mirrorPoint(a, b, target_polygon[i]));
			}

			try {
				//float distance = util::calculatePoLIS(polygon, target_polygon);
				////std::cout << "distance is " << distance << std::endl;
				//return distance;

				float iou = 0.0f;
				if (util::isSimple(polygon) && util::isSimple(target_polygon)){
					iou = util::calculateIOUbyCGAL(polygon, target_polygon);
				}
				else{
					iou = util::calculateIOUbyImage(polygon, target_polygon, 1000);
				}
				//std::cout << "IOU is " << iou << std::endl;
				return iou;
			}
			catch (...) {
				return 0;
			}
		}
	};

protected:
	SymmetryLineDetector();
	~SymmetryLineDetector();

public:
	static std::vector<cv::Point2f> fitSymmetryLine(const std::vector<cv::Point2f>& target_polygon);
};

