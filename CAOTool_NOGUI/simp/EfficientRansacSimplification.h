#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "../util/BuildingLayer.h"
#include "../util/ContourUtils.h"
#include <dlib/optimization.h>
#include <unordered_map>
#include "../efficient_ransac/CurveDetector.h"
#include "../efficient_ransac/LineDetector.h"
#include "../efficient_ransac/PrimitiveShape.h"
#include "../efficient_ransac/ContourGenerator.h"
#include "../efficient_ransac/EfficientRANSAC.h"

namespace simp {

	class EfficientRansacSimplification {

	protected:
		EfficientRansacSimplification() {}

	public:
		static util::Polygon simplify(const util::Polygon& polygon, const std::vector<double>& parameters, float orientation, float min_hole_ratio);
		static void generateContours(util::Polygon polygon, util::Polygon& ans, const std::vector<double>& parameters, float orientation);

	private:
		static std::vector<util::Polygon> findContours(const cv::Mat_<uchar>& img);

	};

}