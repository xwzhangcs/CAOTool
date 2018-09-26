#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "../util/BuildingLayer.h"
#include "../util/ContourUtils.h"

namespace simp {

	class RightAngleSimplification {
	protected:
		RightAngleSimplification() {}

	public:
		static util::Polygon simplify(const util::Polygon& polygon, int resolution, float orientation, float min_hole_ratio, bool optimization);
		static void decomposePolygon(util::Polygon& polygon);

	private:
		static std::tuple<float, int, int> simplifyContour(const util::Ring& contour, util::Ring& result, int resolution, float orientation, bool optimization);
		static double simplifyContour(const util::Ring& contour, util::Ring& result, int resolution, float angle, int dx, int dy, bool refine, bool vertex_refinement);

		static double optimizeVertices(const std::vector<cv::Point>& contour, std::vector<cv::Point>& simplified_contour);
		static std::vector<cv::Point> proposedContour(const std::vector<cv::Point>& contour, std::map<int, int>& x_map, std::map<int, int>& y_map);
		static double optimizeBBox(const std::vector<cv::Point>& contour, std::vector<cv::Point>& simplified_contour);
		static std::vector<cv::Point> proposedBBox(const std::vector<cv::Point>& contour, int x1, int x2, int y1, int y2, int new_x1, int new_x2, int new_y1, int new_y2);

		static std::vector<util::Polygon> findContours(const cv::Mat_<uchar>& img);
		static void refineContour(util::Ring& polygon);
		static void findMaximumRectangle(const std::vector<std::vector<bool>>& grid, const std::vector<float>& x_coords, const std::vector<float>& y_coords, int& x, int& y, int& width, int& height);
	};

}