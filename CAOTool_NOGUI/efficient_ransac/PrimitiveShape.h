#pragma once

#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace efficient_ransac {

	class PrimitiveShape {
	protected:
		/** Supporting points */
		std::vector<cv::Point2f> _points;

		/** Start point index. This index can be a negative value in order to make the indices from start to end contiguous. */
		int _start_index;

		/** Start point that corresponds to the start_index. */
		cv::Point2f _start_point;

		/** End point index. This index can be over the number of points in order to make the indices from start to end contiguous. */
		int _end_index;

		/** End point that corresponds to the end_index. */
		cv::Point2f _end_point;

	public:
		PrimitiveShape() {}
		virtual ~PrimitiveShape() {}

		std::vector<cv::Point2f>& points() { return _points; }
		int startIndex() const { return _start_index; }
		int endIndex() const { return _end_index; }
		cv::Point2f startPoint() const { return _start_point; }
		cv::Point2f endPoint() const { return _end_point; }
		virtual float distance(const cv::Point2f& p) const = 0;
		virtual void setSupportingPoints(const std::vector<cv::Point2f>& supporting_points, const std::vector<int>& supporting_indices) = 0;
	};

}
