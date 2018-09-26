#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "../util/BuildingLayer.h"
#include "../util/ContourUtils.h"
#include <dlib/optimization.h>

namespace simp {

	class CurveSimplification {

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
		CurveSimplification() {}

	public:
		static util::Polygon simplify(const util::Polygon& polygon, float epsilon, float curve_threshold, float orientation, float min_hole_ratio);
		static void decomposePolygon(util::Polygon input, util::Polygon& polygon, float epsilon, float curve_threshold);

	private:
		static std::vector<util::Polygon> findContours(const cv::Mat_<uchar>& img, float epsilon, float curve_threshold);
		static bool approxContour(std::vector<cv::Point2f>& input, util::Polygon &output, float epsilon, float curve_threshold);
		static bool concaveCurve(std::vector<cv::Point2f>& end_points, util::Polygon &polygon);

		static cv::Point3f optimizeByBFGS(const std::vector<cv::Point2d>& points, cv::Point3d init_points);
		static float compute_angle(cv::Point2d a, cv::Point2d b);
		static float compute_abs_angle(cv::Point2d a, cv::Point2d b);
		static float compute_interval(cv::Point2d start, cv::Point2d mid, cv::Point2d end);
		static bool valid_curve(double threshold, const std::vector<cv::Point2d>& points, cv::Point2d center, double radius, cv::Rect bbox);
		static cv::Point2f find_center(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
	};

}