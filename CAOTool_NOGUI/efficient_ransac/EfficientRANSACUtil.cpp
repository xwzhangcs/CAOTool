#include "EfficientRANSACUtil.h"

namespace efficient_ransac {

	/**
	* return angle difference between two angles in range [0, PI/2].
	*/
	float angle_difference(float x, float xi) {
		x = regularize_angle_PI(x);
		xi = regularize_angle_PI(xi);

		if (std::abs(x - xi) <= CV_PI * 0.5) return std::abs(x - xi);
		else return CV_PI - std::abs(x - xi);
	}

	/**
	* return the normalized angle in range [0, PI].
	*/
	float regularize_angle_PI(float x) {
		if (x < 0) {
			x += CV_PI * (int)(-x / CV_PI + 1);
		}
		else {
			x -= CV_PI * (int)(x / CV_PI);
		}

		return x;
	}

	float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segment_only) {
		float r_numerator = (c - a).dot(b - a);
		float r_denomenator = (b - a).dot(b - a);

		if (r_denomenator <= 0.0f) {
			return cv::norm(a - c);
		}

		float r = r_numerator / r_denomenator;

		if (segment_only && (r < 0 || r > 1)) {
			float dist1 = std::hypot(c.x - a.x, c.y - a.y);
			float dist2 = std::hypot(c.x - b.x, c.y - b.y);
			if (dist1 < dist2) {
				return dist1;
			}
			else {
				return dist2;
			}
		}
		else {
			return std::abs((c - a).cross(b - a)) / sqrt(r_denomenator);
		}
	}

}
