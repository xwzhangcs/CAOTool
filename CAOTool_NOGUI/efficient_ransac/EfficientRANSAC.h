#pragma once

#include <vector>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "PrimitiveShape.h"

namespace efficient_ransac {

	class Point {
	public:
		cv::Point2f pos;
		cv::Point2f normal;
		bool used;

	public:
		Point() : used(false) {}
		Point(float x, float y) : pos(x, y), used(false) {}
		Point(float x, float y, float nx, float ny) : pos(x, y), normal(nx, ny), used(false) {}
	};

	class Polygon {
	public:
		std::vector<Point> contour;
		std::vector<std::vector<Point>> holes;

	public:
		Polygon() {}

		void clearUsedFlag() {
			for (auto& pt : contour) pt.used = false;
			for (auto& hole : holes) {
				for (auto& pt : hole) pt.used = false;
			}
		}
	};

	class EfficientRANSAC {
	public:
		EfficientRANSAC();
		~EfficientRANSAC();

	public:
		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> detect(const std::vector<cv::Point2f>& polygon, int curve_num_iterations, int curve_min_points, float curve_max_error_ratio_to_radius, float curve_cluster_epsilon, float curve_min_angle, float curve_min_radius, float curve_max_radius, int line_num_iterations, int line_min_points, float line_max_error, float line_cluster_epsilon, float line_min_length, float line_angle_threshold, std::vector<float>& principal_orientations);

		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> detectCircles(const std::vector<cv::Point2f>& polygon, int curve_num_iterations, int curve_min_points, float curve_max_error_ratio_to_radius, float curve_cluster_epsilon, float curve_min_angle, float curve_min_radius, float curve_max_radius);
		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> detectLines(const std::vector<cv::Point2f>& polygon, int line_num_iterations, int line_min_points, float line_max_error, float line_cluster_epsilon, float line_min_length, float line_angle_threshold, std::vector<float>& principal_orientations, const std::vector<cv::Point2f>& ra_polygon);
	};

}
