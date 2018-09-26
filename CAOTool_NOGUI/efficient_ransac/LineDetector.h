#pragma once

#include <vector>
#include <memory>
#include "EfficientRANSAC.h"

namespace efficient_ransac {


	class Line : public PrimitiveShape {
	private:
		/** Any point on the line */
		cv::Point2f _point;

		/** Orientation vector of the line */
		cv::Point2f _dir;

		/** Length of the line segment */
		float _length;

		bool _right_angle;

	public:
		Line();
		Line(int index, const cv::Point2f& point, const cv::Point2f& dir);
		~Line() {}

		cv::Point2f point() const { return _point; }
		cv::Point2f dir() const { return _dir; }
		void setDir(const cv::Point2f& dir) { _dir = dir; }
		float length() const { return _length; }
		void setRightAngle(bool right_angle) { _right_angle = right_angle; }
		bool isRightAngle() const { return _right_angle; }

		float distance(const cv::Point2f& p) const {
			return std::abs((p - _point).dot(cv::Point2f(_dir.y, -_dir.x)));
		}

		void setSupportingPoints(const std::vector<cv::Point2f>& supporting_points, const std::vector<int>& supporting_indices) {
			_points = supporting_points;

			if (supporting_points.size() == 0) return;

			float _start_pos = std::numeric_limits<float>::max();
			float _end_pos = -std::numeric_limits<float>::max();

			for (auto& pt : supporting_points) {
				float t = (pt - _point).dot(_dir);
				_start_pos = std::min(_start_pos, t);
				_end_pos = std::max(_end_pos, t);
			}

			_start_index = supporting_indices[0];
			_start_point = _point + _dir * (supporting_points[0] - _point).dot(_dir);
			_end_index = supporting_indices.back();
			_end_point = _point + _dir * (supporting_points.back() - _point).dot(_dir);
			_length = _end_pos - _start_pos;
		}
	};

	class LineDetector {
	protected:
		LineDetector() {}

	public:
		static void detect(std::vector<Point>& polygon, int num_iter, int min_points, float max_error, float cluster_epsilon, float min_length, std::vector<float>& principal_angles, float angle_threshold, std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>>& lines);
	};

}