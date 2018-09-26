#include "EfficientRANSAC.h"
#include "CurveDetector.h"
#include "LineDetector.h"

namespace efficient_ransac {

	EfficientRANSAC::EfficientRANSAC() {
	}

	EfficientRANSAC::~EfficientRANSAC() {
	}

	std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> EfficientRANSAC::detect(const std::vector<cv::Point2f>& polygon, int curve_num_iterations, int curve_min_points, float curve_max_error_ratio_to_radius, float curve_cluster_epsilon, float curve_min_angle, float curve_min_radius, float curve_max_radius, int line_num_iterations, int line_min_points, float line_max_error, float line_cluster_epsilon, float line_min_length, float line_angle_threshold, std::vector<float>& principal_orientations) {
		std::vector<Point> points(polygon.size());
		for (int i = 0; i < polygon.size(); i++) {
			points[i] = Point(polygon[i].x, polygon[i].y);
		}

		// detect circles
		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> shapes;
		CurveDetector::detect(points, curve_num_iterations, curve_min_points, curve_max_error_ratio_to_radius, curve_cluster_epsilon, curve_min_angle, curve_min_radius, curve_max_radius, shapes);

		// detect lines based on the principal orientations
		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> results;
		LineDetector::detect(points, line_num_iterations, line_min_points, line_max_error, line_cluster_epsilon, line_min_length, principal_orientations, line_angle_threshold, results);
		shapes.insert(shapes.end(), results.begin(), results.end());

		return shapes;
	}

	std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> EfficientRANSAC::detectCircles(const std::vector<cv::Point2f>& polygon, int curve_num_iterations, int curve_min_points, float curve_max_error_ratio_to_radius, float curve_cluster_epsilon, float curve_min_angle, float curve_min_radius, float curve_max_radius) {
		std::vector<Point> points(polygon.size());
		for (int i = 0; i < polygon.size(); i++) {
			points[i] = Point(polygon[i].x, polygon[i].y);
		}

		// detect circles
		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> shapes;
		CurveDetector::detect(points, curve_num_iterations, curve_min_points, curve_max_error_ratio_to_radius, curve_cluster_epsilon, curve_min_angle, curve_min_radius, curve_max_radius, shapes);

		return shapes;
	}

	std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> EfficientRANSAC::detectLines(const std::vector<cv::Point2f>& polygon, int line_num_iterations, int line_min_points, float line_max_error, float line_cluster_epsilon, float line_min_length, float line_angle_threshold, std::vector<float>& principal_orientations, const std::vector<cv::Point2f>& ra_polygon) {
		std::vector<Point> points(polygon.size());
		for (int i = 0; i < polygon.size(); i++) {
			points[i] = Point(polygon[i].x, polygon[i].y);
		}

		// detect lines based on the principal orientations
		std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>> shapes;
		LineDetector::detect(points, line_num_iterations, line_min_points, line_max_error, line_cluster_epsilon, line_min_length, principal_orientations, line_angle_threshold, shapes);

		return shapes;
	}

}