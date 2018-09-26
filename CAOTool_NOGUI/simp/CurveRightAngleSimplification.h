#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "../util/BuildingLayer.h"
#include "../util/ContourUtils.h"
#include <dlib/optimization.h>
#include <unordered_map>

namespace simp {

	class CurveRightAngleSimplification {

		// In dlib, the general purpose solvers optimize functions that take a column
		// vector as input and return a double.  So here we make a typedef for a
		// variable length column vector of doubles.  This is the type we will use to
		// represent the input to our objective functions which we will be minimizing.
		typedef dlib::matrix<double, 0, 1> column_vector;

		class BFGSSolver {
		public:
			BFGSSolver(const std::vector<cv::Point2d>& points) {
				this->points = points;
			}

			double operator() (const column_vector& arg) const {
				cv::Point2d A0(arg(0), arg(1));
				// compute radius
				double radius = arg(2);
				// compute deviation
				double ans = 0;
				for (int i = 0; i < points.size(); i++){
					ans += pow((pow(cv::norm(A0 - points.at(i)), 2) - radius * radius), 2);
				}
				return ans;
			}

		private:
			std::vector<cv::Point2d> points;
		};

		class BFGSSolverDeriv {
		public:
			BFGSSolverDeriv(const std::vector<cv::Point2d>& points) {
				this->points = points;
			}

			const column_vector operator() (const column_vector& arg) const {
				cv::Point2d A0(arg(0), arg(1));
				float radius = arg(2);
				column_vector ans(3);
				for (int i = 0; i < 3; i++) ans(i) = 0;

				for (int i = 0; i < points.size(); i++) {
					ans(0) += 2 * (pow(cv::norm(points.at(i) - A0), 2) - radius * radius)*(-2 * (points.at(i).x - A0.x));
					ans(1) += 2 * (pow(cv::norm(points.at(i) - A0), 2) - radius * radius)*(-2 * (points.at(i).y - A0.y));
					ans(2) += 2 * (pow(cv::norm(points.at(i) - A0), 2) - radius * radius)*(-2 * radius);
				}

				return ans;
			}

		private:
			std::vector<cv::Point2d> points;
		};

	protected:
		CurveRightAngleSimplification() {}

	public:
		static util::Polygon simplify(const util::Polygon& polygon, float epsilon, float curve_threshold, float angle_threshold, float orientation, float min_hole_ratio);
		static void decomposePolygon(util::Polygon input, util::Polygon& polygon, float epsilon, float curve_threshold, float angle_threshold, float orientation);

	private:
		static std::vector<util::Polygon> findContours(const cv::Mat_<uchar>& img);
		static bool approxContour(std::vector<cv::Point2f>& input, util::Polygon &output, float epsilon, float curve_threshold, float angle_threshold, float orientation);
		static bool concaveCurve(std::vector<cv::Point2f>& end_points, util::Polygon &polygon);

		static cv::Point3f optimizeByBFGS(const std::vector<cv::Point2d>& points, cv::Point3d init_points);
		static float compute_angle(cv::Point2d a, cv::Point2d b);
		static float compute_abs_angle(cv::Point2d a, cv::Point2d b);
		static float compute_interval(cv::Point2d start, cv::Point2d mid, cv::Point2d end);
		static bool valid_curve(double threshold, const std::vector<cv::Point2d>& points, cv::Point2d center, double radius, cv::Rect bbox);
		// new functions
		static float axis_align(const cv::Mat_<uchar>& src);
		static std::vector<cv::Point2f> transform_angle(std::vector<cv::Point2f> contour, cv::Mat_<float> &M, float angle);
		static std::vector<cv::Point2f> transform(std::vector<cv::Point2f> contour, cv::Mat_<float> M);
		static std::vector<cv::Point2f> rectify_curves(std::vector<cv::Point2f> contour, std::vector<std::vector<cv::Point2f>> curves, std::vector<std::vector<cv::Point2f>> curves_attrs, std::vector<std::vector<cv::Point2f>>& curves_rectify, std::vector<std::vector<cv::Point2f>>& curves_attrs_rectify, float snap_threshold);
		static std::vector<cv::Point2f> del_redundant_points(std::vector<cv::Point2f> contour);
		static std::vector<cv::Point2f> contour_rectify(std::vector<cv::Point2f>& contour, float threshold, float epsilon);
		static std::vector<cv::Point2f> contour_rectify_no_curve(std::vector<cv::Point2f>& contour, float threshold, float epsilon);
		static cv::Point2f find_center(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);

	};

}