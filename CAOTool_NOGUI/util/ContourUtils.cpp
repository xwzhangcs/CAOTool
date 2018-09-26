#include "ContourUtils.h"
#include <iostream>
#include <boost/polygon/polygon.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <list>

namespace util {
	
	PrimitiveRectangle::PrimitiveRectangle(const cv::Mat_<float>& mat, const cv::Point2f& min_pt, const cv::Point2f& max_pt) {
		this->mat = mat;
		this->min_pt = min_pt;
		this->max_pt = max_pt;
	}

	boost::shared_ptr<PrimitiveShape> PrimitiveRectangle::clone() const {
		boost::shared_ptr<PrimitiveShape> ans = boost::shared_ptr<PrimitiveShape>(new PrimitiveRectangle(mat, min_pt, max_pt));
		return ans;
	}

	std::vector<cv::Point2f> PrimitiveRectangle::getActualPoints() const {
		std::vector<cv::Point2f> ans(4);

		cv::Mat_<float> p0 = (cv::Mat_<float>(3, 1) << min_pt.x, min_pt.y, 1);
		cv::Mat_<float> q0 = mat * p0;
		cv::Mat_<float> p1 = (cv::Mat_<float>(3, 1) << max_pt.x, min_pt.y, 1);
		cv::Mat_<float> q1 = mat * p1;
		cv::Mat_<float> p2 = (cv::Mat_<float>(3, 1) << max_pt.x, max_pt.y, 1);
		cv::Mat_<float> q2 = mat * p2;
		cv::Mat_<float> p3 = (cv::Mat_<float>(3, 1) << min_pt.x, max_pt.y, 1);
		cv::Mat_<float> q3 = mat * p3;

		ans[0] = cv::Point2f(q0(0, 0), q0(1, 0));
		ans[1] = cv::Point2f(q1(0, 0), q1(1, 0));
		ans[2] = cv::Point2f(q2(0, 0), q2(1, 0));
		ans[3] = cv::Point2f(q3(0, 0), q3(1, 0));

		return ans;
	}

	int PrimitiveRectangle::type() const {
		return TYPE_RECTANGLE;
	}

	PrimitiveTriangle::PrimitiveTriangle(const cv::Mat_<float>& mat) {
		this->mat = mat;
	}

	PrimitiveTriangle::PrimitiveTriangle(const cv::Mat_<float>& mat, const std::vector<cv::Point2f>& points) {
		this->mat = mat;
		this->points = points;
	}

	boost::shared_ptr<PrimitiveShape> PrimitiveTriangle::clone() const {
		boost::shared_ptr<PrimitiveShape> ans = boost::shared_ptr<PrimitiveShape>(new PrimitiveTriangle(mat, points));
		return ans;
	}

	std::vector<cv::Point2f> PrimitiveTriangle::getActualPoints() const {
		std::vector<cv::Point2f> ans(points.size());

		for (int i = 0; i < points.size(); i++) {
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << points[i].x, points[i].y, 1);
			cv::Mat_<float> q = mat * p;
			ans[i] = cv::Point2f(q(0, 0), q(1, 0));
		}
		return ans;
	}

	int PrimitiveTriangle::type() const {
		return TYPE_TRIANGLE;
	}

	PrimitiveCurve::PrimitiveCurve(const cv::Mat_<float>& mat, float theta_start, float theta_end, const cv::Point2f &center, float radius) {
		this->mat = mat;
		this->theta_start = theta_start;
		this->theta_end = theta_end;
		this->center = center;
		this->radius = radius;
		if (std::isnan(theta_start) || std::isnan(theta_end) || std::isnan(radius)) throw "Invalid angle or radius";
		if (theta_start == theta_end) throw "Two angles have to be different";
	}

	boost::shared_ptr<PrimitiveShape> PrimitiveCurve::clone() const {
		boost::shared_ptr<PrimitiveShape> ans = boost::shared_ptr<PrimitiveShape>(new PrimitiveCurve(mat, theta_start, theta_end, center, radius));
		return ans;
	}

	std::vector<cv::Point2f> PrimitiveCurve::getActualPoints() const {
		float angle_start_end = this->theta_end - this->theta_start;

		float angle_between = 5;
		int num_points = std::ceil(abs(angle_start_end / 5));
		angle_between = angle_start_end / num_points;
		num_points = num_points + 1;
		std::vector<cv::Point2f> ans(num_points);
		for (int k = 0; k < num_points - 1; k++) {
			double x = abs(this->radius) * cos(CV_PI * (this->theta_start + angle_between * k) / 180) + this->center.x;
			double y = abs(this->radius) * sin(CV_PI * (this->theta_start + angle_between * k) / 180) + this->center.y;
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << x, y, 1);
			cv::Mat_<float> q = mat * p;
			ans[k] = cv::Point2f(q(0, 0), q(1, 0));
		}

		// for the last point
		{
			double x = abs(this->radius) * cos(CV_PI * (this->theta_end) / 180) + this->center.x;
			double y = abs(this->radius) * sin(CV_PI * (this->theta_end) / 180) + this->center.y;
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << x, y, 1);
			cv::Mat_<float> q = mat * p;
			ans.back() = cv::Point2f(q(0, 0), q(1, 0));
		}

		return ans;
	}

	int PrimitiveCurve::type() const {
		return TYPE_CURVE;
	}

	Ring::Ring() {
		mat = cv::Mat_<float>::eye(3, 3);
	}

	Ring::Ring(const Ring& ring) {
		this->mat = ring.mat.clone();
		this->points = ring.points;
		this->pointsType = ring.pointsType;
	}

	Ring::Ring(const std::vector<cv::Point2f>& points) {
		mat = cv::Mat_<float>::eye(3, 3);
		this->points = points;
	}

	Ring& Ring::operator=(const Ring& ring) {
		this->mat = ring.mat.clone();
		this->points = ring.points;
		this->pointsType = ring.pointsType;
		return *this;
	}

	Ring& Ring::operator=(const std::vector<cv::Point2f>& points) {
		this->points = points;
		return *this;
	}

	const cv::Point2f& Ring::front() const {
		return points.front();
	}

	cv::Point2f& Ring::front() {
		return points.front();
	}

	const cv::Point2f& Ring::back() const {
		return points.back();
	}

	cv::Point2f& Ring::back() {
		return points.back();
	}

	std::vector<cv::Point2f>::iterator Ring::begin() {
		return points.begin();
	}

	std::vector<cv::Point2f>::iterator Ring::end() {
		return points.end();
	}

	size_t Ring::size() const {
		return points.size();
	}

	void Ring::clear() {
		points.clear();
	}

	void Ring::resize(size_t s) {
		points.resize(s);
	}

	const cv::Point2f& Ring::operator[](int index) const {
		return points[index];
	}

	cv::Point2f& Ring::operator[](int index) {
		return points[index];
	}

	void Ring::push_back(const cv::Point2f& pt) {
		points.push_back(pt);
	}

	void Ring::pop_back() {
		points.pop_back();
	}

	void Ring::erase(std::vector<cv::Point2f>::iterator position) {
		points.erase(position);
	}

	void Ring::translate(float x, float y) {
		for (int i = 0; i < points.size(); i++) {
			points[i].x += x;
			points[i].y += y;
		}
	}

	void Ring::transform(const cv::Mat_<float>& m) {
		mat = m * mat;
		/*
		for (int i = 0; i < points.size(); i++) {
			cv::Mat_<float> p = m * (cv::Mat_<float>(3, 1) << points[i].x, points[i].y, 1);
			points[i].x = p(0, 0);
			points[i].y = p(1, 0);
		}
		*/
	}

	void Ring::clockwise() {
		if (!isClockwise(points)) {
			std::reverse(points.begin(), points.end());
		}
	}

	void Ring::counterClockwise() {
		if (isClockwise(points)) {
			std::reverse(points.begin(), points.end());
		}
	}

	cv::Point2f Ring::getActualPoint(int index) const {
		cv::Mat_<float> pt = (cv::Mat_<float>(3, 1) << points[index].x, points[index].y, 1);
		cv::Mat_<float> pt2 = mat * pt;

		cv::Point2f ans;
		ans.x = pt2(0, 0);
		ans.y = pt2(1, 0);
		return ans;
	}

	Ring Ring::getActualPoints() const {
		Ring ans;
		ans.points.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			cv::Mat_<float> pt = (cv::Mat_<float>(3, 1) << points[i].x, points[i].y, 1);
			cv::Mat_<float> pt2 = mat * pt;
			ans.points[i].x = pt2(0, 0);
			ans.points[i].y = pt2(1, 0);
		}
		return ans;
	}
	
	bool Ring::isSimple() const {
		CDT cdt;
		CGAL::Partition_traits_2<Kernel>::Polygon_2 polygon;
		for (int i = 0; i < points.size(); i++) {
			polygon.push_back(CDT::Point(points[i].x, points[i].y));
		}
		return polygon.is_simple();
	}

	Polygon::Polygon() {
		mat = cv::Mat_<float>::eye(3, 3);
	}

	Polygon Polygon::clone() const {
		Polygon ans;
		ans.mat = mat;
		ans.contour = contour;
		ans.holes = holes;

		ans.primitive_shapes.resize(primitive_shapes.size());
		for (int i = 0; i < primitive_shapes.size(); i++) {
			ans.primitive_shapes[i] = primitive_shapes[i]->clone();
		}

		return ans;
	}

	void Polygon::translate(float x, float y) {
		contour.translate(x, y);
		for (int i = 0; i < holes.size(); i++) {
			holes[i].translate(x, y);
		}
	}

	void Polygon::transform(const cv::Mat_<float>& m) {
		contour.transform(m);
		for (int i = 0; i < holes.size(); i++) {
			holes[i].transform(m);
		}
	}

	void Polygon::clockwise() {
		contour.clockwise();
		for (int i = 0; i < holes.size(); i++) {
			holes[i].clockwise();
		}
	}

	void Polygon::counterClockwise() {
		contour.counterClockwise();
		for (int i = 0; i < holes.size(); i++) {
			holes[i].counterClockwise();
		}
	}

	bool isClockwise(const std::vector<cv::Point2f>& polygon) {
		float S = 0;
		for (int i = 0; i < polygon.size(); i++) {
			int next = (i + 1) % polygon.size();
			S += (polygon[next].x - polygon[i].x) * (polygon[next].y + polygon[i].y);
		}
		return S > 0;
	}

	void clockwise(std::vector<cv::Point2f>& polygon) {
		if (!isClockwise(polygon)) {
			std::reverse(polygon.begin(), polygon.end());
		}
	}

	void counterClockwise(std::vector<cv::Point2f>& polygon) {
		if (isClockwise(polygon)) {
			std::reverse(polygon.begin(), polygon.end());
		}
	}

	bool isSimple(const Polygon& polygon) {
		if (polygon.contour.size() == 0) return false;

		CGAL::Polygon_2<Kernel> pgn;
		std::vector<cv::Point2f> contour = polygon.contour.getActualPoints().points;
		for (int i = 0; i < contour.size(); i++) {
			pgn.push_back(Kernel::Point_2(contour[i].x, contour[i].y));
		}
		if (!pgn.is_simple()) return false;

		for (int i = 0; i < polygon.holes.size(); i++) {
			if (polygon.holes[i].size() == 0) continue;

			CGAL::Polygon_2<Kernel> hl;
			std::vector<cv::Point2f> hole = polygon.holes[i].getActualPoints().points;
			for (int j = 0; j < hole.size(); j++) {
				hl.push_back(Kernel::Point_2(hole[j].x, hole[j].y));
			}
			if (!hl.is_simple()) return false;
		}
		
		for (int i = 0; i < contour.size(); i++) {
			int i2 = (i + 1) % contour.size();

			for (int j = 0; j < polygon.holes.size(); j++) {
				std::vector<cv::Point2f> hole = polygon.holes[j].getActualPoints().points;

				for (int k = 0; k < hole.size(); k++) {
					int k2 = (k + 1) % hole.size();

					if (isTangent(contour[i], contour[i2], hole[k], hole[k2])) return false;
					double tab, tcd;
					cv::Point2f intPt;
					if (segmentSegmentIntersection(contour[i], contour[i2], hole[k], hole[k2], &tab, &tcd, true, intPt)) return false;
				}
			}
		}
		for (int i = 0; i < polygon.holes.size(); i++) {
			std::vector<cv::Point2f> hole = polygon.holes[i].getActualPoints().points;

			for (int j = i + 1; j < polygon.holes.size(); j++) {
				std::vector<cv::Point2f> hole2 = polygon.holes[j].getActualPoints().points;

				for (int k = 0; k < hole.size(); k++) {
					int k2 = (k + 1) % hole.size();
					for (int l = 0; l < hole2.size(); l++) {
						int l2 = (l + 1) % hole2.size();

						if (isTangent(hole[k], hole[k2], hole2[l], hole2[l2])) return false;
						double tab, tcd;
						cv::Point2f intPt;
						if (segmentSegmentIntersection(hole[k], hole[k2], hole2[l], hole2[l2], &tab, &tcd, true, intPt)) return false;
					}
				}
			}
		}

		return true;
	}

	bool isSimple(const Ring& points) {
		CDT cdt;
		CGAL::Partition_traits_2<Kernel>::Polygon_2 polygon;
		for (int i = 0; i < points.size(); i++) {
			polygon.push_back(CDT::Point(points[i].x, points[i].y));
		}
		return polygon.is_simple();
	}

	bool isSimple(const std::vector<cv::Point>& points) {
		CDT cdt;
		CGAL::Partition_traits_2<Kernel>::Polygon_2 polygon;
		for (int i = 0; i < points.size(); i++) {
			polygon.push_back(CDT::Point(points[i].x, points[i].y));
		}
		return polygon.is_simple();
	}

	void transform(std::vector<cv::Point2f>& polygon, const cv::Mat_<float>& m) {
		for (int i = 0; i < polygon.size(); i++) {
			cv::Mat_<float> p = m * (cv::Mat_<float>(3, 1) << polygon[i].x, polygon[i].y, 1);
			polygon[i].x = p(0, 0);
			polygon[i].y = p(1, 0);
		}
	}

	/**
	 * Remove the redundant points that have the same coordinates, and remove the point that are collinear to the adjacent points.
	 */
	std::vector<cv::Point> removeRedundantPoint(const std::vector<cv::Point>& polygon) {
		std::vector<cv::Point> ans;
		if (polygon.size() == 0) return ans;

		ans.push_back(polygon[0]);
		for (int i = 1; i < polygon.size(); i++) {
			if (polygon[i] != polygon[i - 1]) {
				ans.push_back(polygon[i]);
			}
		}
		if (ans.size() > 1 && ans.back() == ans.front()) ans.pop_back();

		for (int i = 0; i < ans.size() && ans.size() >= 3; ) {
			int prev = (i - 1 + ans.size()) % ans.size();
			int next = (i + 1) % ans.size();
			if (dotProduct(ans[i] - ans[prev], ans[next] - ans[i]) > 0 && std::abs(crossProduct(ans[i] - ans[prev], ans[next] - ans[i])) < 0.0001) {
				ans.erase(ans.begin() + i);
			}
			else {
				i++;
			}
		}

		return ans;
	}

	Ring removeRedundantPoint(const Ring& polygon) {
		Ring ans;
		if (polygon.size() == 0) return ans;

		ans.push_back(polygon[0]);
		for (int i = 1; i < polygon.size(); i++) {
			if (polygon[i] != polygon[i - 1]) {
				ans.push_back(polygon[i]);
			}
		}
		if (ans.size() > 1 && ans.back() == ans.front()) ans.pop_back();

		for (int i = 0; i < ans.size() && ans.size() >= 3;) {
			int prev = (i - 1 + ans.size()) % ans.size();
			int next = (i + 1) % ans.size();
			if (dotProduct(ans[i] - ans[prev], ans[next] - ans[i]) > 0 && std::abs(crossProduct(ans[i] - ans[prev], ans[next] - ans[i])) < 0.0001) {
				ans.erase(ans.begin() + i);
			}
			else {
				i++;
			}
		}

		return ans;
	}

	Ring resolveSelfIntersection(const Ring& ring) {
		cv::Rect rect = util::boundingBox(ring.points);
		
		std::vector<cv::Point> contour(ring.size());
		for (int i = 0; i < ring.size(); i++) {
			contour[i] = cv::Point(ring[i].x, ring[i].y);
		}

		cv::Mat_<uchar> img;
		util::createImageFromContour(rect.width, rect.height, contour, cv::Point(-rect.x, -rect.y), img);
		std::vector<util::Polygon> polygons = util::findContours(img, false);

		// If there are diagonal connections, we need to force to connect them using 4-neighbors because they are supposed to be a single contour
		if (polygons.size() > 1) {
			for (int r = 0; r < img.rows - 1; r++) {
				for (int c = 0; c < img.cols - 1; c++) {
					if (img(r, c) == 255 && img(r, c + 1) == 0 && img(r + 1, c) == 0 && img(r + 1, c + 1) == 255) {
						img(r, c + 1) = 255;
					}
					else if (img(r, c) == 0 && img(r, c + 1) == 255 && img(r + 1, c) == 255 && img(r + 1, c + 1) == 0) {
						img(r, c) = 255;
					}
				}
			}
		}
		
		Ring ans = polygons[0].contour;
		ans.mat = ring.mat;
		ans.translate(rect.x, rect.y);
		return ans;
	}

	cv::Rect boundingBox(const std::vector<cv::Point>& polygon) {
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();
		int max_y = -std::numeric_limits<int>::max();
		for (int i = 0; i < polygon.size(); i++) {
			min_x = std::min(min_x, polygon[i].x);
			max_x = std::max(max_x, polygon[i].x);
			min_y = std::min(min_y, polygon[i].y);
			max_y = std::max(max_y, polygon[i].y);
		}

		return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
	}

	cv::Rect boundingBox(const std::vector<cv::Point2f>& polygon) {
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();
		int max_y = -std::numeric_limits<int>::max();
		for (int i = 0; i < polygon.size(); i++) {
			min_x = std::min(min_x, (int)polygon[i].x);
			max_x = std::max(max_x, (int)polygon[i].x);
			min_y = std::min(min_y, (int)polygon[i].y);
			max_y = std::max(max_y, (int)polygon[i].y);
		}

		return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
	}

	cv::Rect calculateOBB(const std::vector<cv::Point2f>& polygon, cv::Mat_<float>& trans_mat) {
		cv::Rect ans(0, 0, 0, 0);
		trans_mat = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1);
		int min_area = std::numeric_limits<int>::max();

		for (int i = 0; i < polygon.size(); i++) {
			int next = (i + 1) % polygon.size();
			if (length(polygon[next] - polygon[i]) == 0) continue;

			// calculate the angle of the edge
			float theta = std::atan2(polygon[next].y - polygon[i].y, polygon[next].x - polygon[i].x);
			cv::Mat_<float> mat = (cv::Mat_<float>(2, 2) << std::cos(theta), std::sin(theta), -std::sin(theta), std::cos(theta));
			
			// rotate the polygon
			std::vector<cv::Point2f> rotated_polygon(polygon.size());
			for (int j = 0; j < polygon.size(); j++) {
				cv::Mat_<float> p = (cv::Mat_<float>(2, 1) << polygon[j].x, polygon[j].y);
				cv::Mat_<float> p2 = mat * p;
				rotated_polygon[j] = cv::Point2f(p2(0, 0), p2(1, 0));
			}

			// calculate the bounding box
			cv::Rect rect = boundingBox(rotated_polygon);
			if (rect.width * rect.height < min_area) {
				min_area = rect.width * rect.height;
				ans = rect;
				trans_mat = mat;
			}
		}

		return ans;
	}

	bool withinPolygon(const cv::Point2f& pt, const Polygon& polygon) {
		if (!withinPolygon(pt, polygon.contour)) return false;

		for (int i = 0; i < polygon.holes.size(); i++) {
			if (withinPolygon(pt, polygon.holes[i])) return false;
		}

		return true;
	}

	bool withinPolygon(const cv::Point2f& pt, const Ring& ring) {
		boost::geometry::model::ring<boost::geometry::model::d2::point_xy<float>> contour;
		for (int i = 0; i < ring.size(); ++i) {
			contour.push_back(boost::geometry::model::d2::point_xy<float>(ring[i].x, ring[i].y));
		}
		boost::geometry::correct(contour);

		return boost::geometry::within(boost::geometry::model::d2::point_xy<float>(pt.x, pt.y), contour);
	}

	bool withinPolygon(const Ring& inside_ring, const Ring& outside_ring) {
		boost::geometry::model::ring<boost::geometry::model::d2::point_xy<float>> inside_contour;
		for (int i = 0; i < inside_ring.size(); ++i) {
			inside_contour.push_back(boost::geometry::model::d2::point_xy<float>(inside_ring[i].x, inside_ring[i].y));
		}
		boost::geometry::correct(inside_contour);
		
		boost::geometry::model::ring<boost::geometry::model::d2::point_xy<float>> outside_contour;
		for (int i = 0; i < outside_ring.size(); ++i) {
			outside_contour.push_back(boost::geometry::model::d2::point_xy<float>(outside_ring[i].x, outside_ring[i].y));
		}
		boost::geometry::correct(outside_contour);

		return boost::geometry::within(inside_contour, outside_contour);
	}

	/**
	* Calculate the intersection over union (IOU) inside the specified roi.
	* The images have to be CV_8U type.
	*/
	double calculateIOU(const cv::Mat_<uchar>& img, const cv::Mat_<uchar>& img2) {
		double union_cnt = 0;
		double inter_cnt = 0;
		for (int r = 0; r < img.rows; r++) {
			for (int c = 0; c < img.cols; c++) {
				union_cnt += std::max(img.at<uchar>(r, c), img2.at<uchar>(r, c));
				inter_cnt += std::min(img.at<uchar>(r, c), img2.at<uchar>(r, c));
			}
		}

		return (double)inter_cnt / union_cnt;
	}

	/**
	* Calculate the intersection over union (IOU) inside the specified roi.
	* The images have to be CV_8U type.
	* Check the IOU only within the specified region, rect.
	*/
	double calculateIOU(const cv::Mat_<uchar>& img1, const cv::Mat_<uchar>& img2, const cv::Rect& rect) {
		double union_cnt = 0;
		double inter_cnt = 0;

		for (int r = rect.y; r < rect.y + rect.height; r++) {
			for (int c = rect.x; c < rect.x + rect.width; c++) {
				union_cnt += std::max(img1.at<uchar>(r, c), img2.at<uchar>(r, c));
				inter_cnt += std::min(img1.at<uchar>(r, c), img2.at<uchar>(r, c));
			}
		}

		return (float)inter_cnt / union_cnt;
	}

	/**
	 * Calculate the intersection over union (IOU) between two polygons.
	 * Since calculating the polygon intersecion and union exactly is very expensive,
	 * we resort to an image-based approach that can quickly calculate the approximate IOU.
	 */
	double calculateIOU(const Polygon& polygon1, const Polygon& polygon2) {
		Ring contour1 = polygon1.contour.getActualPoints();
		Ring contour2 = polygon2.contour.getActualPoints();

		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		for (int i = 0; i < contour1.size(); i++) {
			min_x = std::min(min_x, (int)contour1[i].x);
			min_y = std::min(min_y, (int)contour1[i].y);
			max_x = std::max(max_x, (int)(contour1[i].x + 0.5));
			max_y = std::max(max_y, (int)(contour1[i].y + 0.5));
		}
		for (int i = 0; i < contour2.size(); i++) {
			min_x = std::min(min_x, (int)contour2[i].x);
			min_y = std::min(min_y, (int)contour2[i].y);
			max_x = std::max(max_x, (int)(contour2[i].x + 0.5));
			max_y = std::max(max_y, (int)(contour2[i].y + 0.5));
		}

		cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);
		
		std::vector<std::vector<cv::Point>> contour_points1(1 + polygon1.holes.size());
		contour_points1[0].resize(contour1.size());
		for (int i = 0; i < contour1.size(); i++) {
			contour_points1[0][i] = cv::Point(contour1[i].x - min_x, contour1[i].y - min_y);
		}
		for (int i = 0; i < polygon1.holes.size(); i++) {
			Ring hole = polygon1.holes[i].getActualPoints();
			contour_points1[i + 1].resize(hole.size());
			for (int j = 0; j < hole.size(); j++) {
				contour_points1[i + 1][j] = cv::Point(hole[j].x - min_x, hole[j].y - min_y);
			}
		}
		cv::fillPoly(img1, contour_points1, cv::Scalar(255), cv::LINE_4);

		cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);

		std::vector<std::vector<cv::Point>> contour_points2(1 + polygon2.holes.size());
		contour_points2[0].resize(contour2.size());
		for (int i = 0; i < contour2.size(); i++) {
			contour_points2[0][i] = cv::Point(contour2[i].x - min_x, contour2[i].y - min_y);
		}
		for (int i = 0; i < polygon2.holes.size(); i++) {
			Ring hole = polygon2.holes[i].getActualPoints();
			contour_points2[i + 1].resize(hole.size());
			for (int j = 0; j < hole.size(); j++) {
				contour_points2[i + 1][j] = cv::Point(hole[j].x - min_x, hole[j].y - min_y);
			}
		}
		cv::fillPoly(img2, contour_points2, cv::Scalar(255), cv::LINE_4);

		int inter_cnt = 0;
		int union_cnt = 0;
		for (int r = 0; r < img1.rows; r++) {
			for (int c = 0; c < img1.cols; c++) {
				if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
				if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
			}
		}

		return (double)inter_cnt / union_cnt;
	}

	double calculateIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		for (int i = 0; i < polygon1.size(); i++) {
			min_x = std::min(min_x, (int)polygon1[i].x);
			min_y = std::min(min_y, (int)polygon1[i].y);
			max_x = std::max(max_x, (int)(polygon1[i].x + 0.5));
			max_y = std::max(max_y, (int)(polygon1[i].y + 0.5));
		}
		for (int i = 0; i < polygon2.size(); i++) {
			min_x = std::min(min_x, (int)polygon2[i].x);
			min_y = std::min(min_y, (int)polygon2[i].y);
			max_x = std::max(max_x, (int)(polygon2[i].x + 0.5));
			max_y = std::max(max_y, (int)(polygon2[i].y + 0.5));
		}

		cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);

		std::vector<std::vector<cv::Point>> contour_points1(1);
		contour_points1[0].resize(polygon1.size());
		for (int i = 0; i < polygon1.size(); i++) {
			contour_points1[0][i] = cv::Point(polygon1[i].x - min_x, polygon1[i].y - min_y);
		}
		cv::fillPoly(img1, contour_points1, cv::Scalar(255), cv::LINE_4);

		cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);

		std::vector<std::vector<cv::Point>> contour_points2(1);
		contour_points2[0].resize(polygon2.size());
		for (int i = 0; i < polygon2.size(); i++) {
			contour_points2[0][i] = cv::Point(polygon2[i].x - min_x, polygon2[i].y - min_y);
		}
		cv::fillPoly(img2, contour_points2, cv::Scalar(255), cv::LINE_4);

		int inter_cnt = 0;
		int union_cnt = 0;
		for (int r = 0; r < img1.rows; r++) {
			for (int c = 0; c < img1.cols; c++) {
				if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
				if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
			}
		}

		return (double)inter_cnt / union_cnt;
	}

	double calculateIOUbyCGAL(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
		if (polygon1.size() == 0 || polygon2.size() == 0) return 0;

		CGAL::Polygon_2<Kernel> pol1;
		for (auto& pt : polygon1) {
			pol1.push_back(Kernel::Point_2(pt.x, pt.y));
		}
		if (!pol1.is_simple()) {
			throw "calculateIOU: polygon1 is not simple.";
		}
		if (pol1.is_clockwise_oriented()) pol1.reverse_orientation();

		CGAL::Polygon_2<Kernel> pol2;
		for (auto& pt : polygon2) {
			pol2.push_back(Kernel::Point_2(pt.x, pt.y));
		}
		if (!pol2.is_simple()) {
			throw "calculateIOU: polygon2 is not simple.";
		}
		if (pol2.is_clockwise_oriented()) pol2.reverse_orientation();

		if (CGAL::do_intersect(pol1, pol2)) {
			std::list<CGAL::Polygon_with_holes_2<Kernel>> inter;
			CGAL::intersection(pol1, pol2, std::back_inserter(inter));

			CGAL::Polygon_with_holes_2<Kernel> uni;
			CGAL::join(pol1, pol2, uni);

			float inter_area = 0;
			for (auto it = inter.begin(); it != inter.end(); it++) {
				inter_area += area(*it);
			}

			return (double)inter_area / area(uni);
		}
		else {
			return 0;
		}
	}

	double calculateIOUbyImage(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2, int image_size) {
		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		for (int i = 0; i < polygon1.size(); i++) {
			min_x = std::min(min_x, (int)polygon1[i].x);
			min_y = std::min(min_y, (int)polygon1[i].y);
			max_x = std::max(max_x, (int)(polygon1[i].x + 0.5));
			max_y = std::max(max_y, (int)(polygon1[i].y + 0.5));
		}
		for (int i = 0; i < polygon2.size(); i++) {
			min_x = std::min(min_x, (int)polygon2[i].x);
			min_y = std::min(min_y, (int)polygon2[i].y);
			max_x = std::max(max_x, (int)(polygon2[i].x + 0.5));
			max_y = std::max(max_y, (int)(polygon2[i].y + 0.5));
		}

		float scale = (float)image_size / std::max(max_x - min_x, max_y - min_y);

		cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(image_size, image_size);
		if (polygon1.size() > 0) {
			std::vector<std::vector<cv::Point>> contour_points1(1);
			contour_points1[0].resize(polygon1.size());
			for (int i = 0; i < polygon1.size(); i++) {
				contour_points1[0][i] = cv::Point((polygon1[i].x - min_x) * scale, (polygon1[i].y - min_y) * scale);
			}
			cv::fillPoly(img1, contour_points1, cv::Scalar(255), cv::LINE_4);
		}

		cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(image_size, image_size);
		if (polygon2.size() > 0) {
			std::vector<std::vector<cv::Point>> contour_points2(1);
			contour_points2[0].resize(polygon2.size());
			for (int i = 0; i < polygon2.size(); i++) {
				contour_points2[0][i] = cv::Point((polygon2[i].x - min_x) * scale, (polygon2[i].y - min_y) * scale);
			}
			cv::fillPoly(img2, contour_points2, cv::Scalar(255), cv::LINE_4);
		}

		int inter_cnt = 0;
		int union_cnt = 0;
		for (int r = 0; r < img1.rows; r++) {
			for (int c = 0; c < img1.cols; c++) {
				if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
				if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
			}
		}
		if (union_cnt == 0){
			return 0;
		}
		return (double)inter_cnt / union_cnt;
	}

	double calculateIOU(const std::vector<Polygon>& polygons1, const std::vector<Polygon>& polygons2) {
		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		for (auto& polygon : polygons1) {
			Ring contour1 = polygon.contour.getActualPoints();
			for (auto& pt : contour1) {
				min_x = std::min(min_x, (int)pt.x);
				min_y = std::min(min_y, (int)pt.y);
				max_x = std::max(max_x, (int)(pt.x + 0.5));
				max_y = std::max(max_y, (int)(pt.y + 0.5));
			}
		}
		for (auto& polygon : polygons2) {
			Ring contour2 = polygon.contour.getActualPoints();
			for (auto& pt : contour2) {
				min_x = std::min(min_x, (int)pt.x);
				min_y = std::min(min_y, (int)pt.y);
				max_x = std::max(max_x, (int)(pt.x + 0.5));
				max_y = std::max(max_y, (int)(pt.y + 0.5));
			}
		}

		cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);
		for (auto& polygon : polygons1) {
			std::vector<std::vector<cv::Point>> contour_points(1 + polygon.holes.size());
			Ring contour = polygon.contour.getActualPoints();
			contour_points[0].resize(contour.size());
			for (int i = 0; i < contour.size(); i++) {
				contour_points[0][i] = cv::Point(contour[i].x - min_x, contour[i].y - min_y);
			}
			for (int i = 0; i < polygon.holes.size(); i++) {
				Ring hole = polygon.holes[i].getActualPoints();
				contour_points[i + 1].resize(hole.size());
				for (int j = 0; j < hole.size(); j++) {
					contour_points[i + 1][j] = cv::Point(hole[j].x - min_x, hole[j].y - min_y);
				}
			}
			cv::fillPoly(img1, contour_points, cv::Scalar(255), cv::LINE_4);
		}

		cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);
		for (auto& polygon : polygons2) {
			std::vector<std::vector<cv::Point>> contour_points(1 + polygon.holes.size());
			Ring contour = polygon.contour.getActualPoints();
			contour_points[0].resize(contour.size());
			for (int i = 0; i < contour.size(); i++) {
				contour_points[0][i] = cv::Point(contour[i].x - min_x, contour[i].y - min_y);
			}
			for (int i = 0; i < polygon.holes.size(); i++) {
				Ring hole = polygon.holes[i].getActualPoints();
				contour_points[i + 1].resize(hole.size());
				for (int j = 0; j < hole.size(); j++) {
					contour_points[i + 1][j] = cv::Point(hole[j].x - min_x, hole[j].y - min_y);
				}
			}
			cv::fillPoly(img2, contour_points, cv::Scalar(255), cv::LINE_4);
		}

		int inter_cnt = 0;
		int union_cnt = 0;
		for (int r = 0; r < img1.rows; r++) {
			for (int c = 0; c < img1.cols; c++) {
				if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
				if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
			}
		}

		return (double)inter_cnt / union_cnt;
	}

	double calculateArea(const Polygon& polygon) {
		double ans = cv::contourArea(polygon.contour.points);
		for (int i = 0; i < polygon.holes.size(); i++) {
			ans -= cv::contourArea(polygon.holes[i].points);
		}
		return ans;
	}

	double calculateArea(const cv::Mat_<uchar>& img) {
		return cv::sum(img)[0] / 255;
	}

	double area(const CGAL::Polygon_with_holes_2<Kernel>& pwh) {
		double ans = 0;

		ans += CGAL::to_double(pwh.outer_boundary().area());
		for (auto it = pwh.holes_begin(); it != pwh.holes_end(); it++) {
			ans -= CGAL::to_double(it->area());
		}

		return ans;
	}

	double area(const std::vector<cv::Point2f>& polygon) {
		CGAL::Polygon_2<Kernel> pgn;
		for (int i = 0; i < polygon.size(); i++) {
			pgn.push_back(Kernel::Point_2(polygon[i].x, polygon[i].y));
		}
		return std::abs(CGAL::to_double(pgn.area()));
	}

	/**
	 * Helper function to extract contours from the input image.
	 * The image has to be of type CV_8U, and has values either 0 or 255.
	 * Note that the input image is not modified by this function.
	 */
	std::vector<Polygon> findContours(const cv::Mat_<uchar>& img, bool add_right_corner) {
		std::vector<Polygon> ans;

		cv::Mat_<uchar> img2 = img.clone();
		while (true) {
			bool updated = false;
			for (int r = 0; r < img.rows - 1; r++) {
				for (int c = 0; c < img.cols - 1; c++) {
					if (img2(r, c) == 255 && img2(r + 1, c + 1) == 255 && img2(r + 1, c) == 0 && img2(r, c + 1) == 0) {
						updated = true;
						img2(r + 1, c) = 255;
					}
					else if (img2(r, c) == 0 && img2(r + 1, c + 1) == 0 && img2(r + 1, c) == 255 && img2(r, c + 1) == 255) {
						updated = true;
						img2(r, c) = 255;
					}
				}
			}
			if (!updated) break;
		}

		// resize x4
		cv::Mat_<uchar> img3;
		cv::resize(img2, img3, cv::Size(img.cols * 4, img.rows * 4), 0, 0, cv::INTER_NEAREST);

		// add padding
		cv::Mat_<uchar> padded = cv::Mat_<uchar>::zeros(img3.rows + 1, img3.cols + 1);
		img3.copyTo(padded(cv::Rect(0, 0, img3.cols, img3.rows)));

		// dilate image
		cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 1, 1, 0, 1, 1, 0, 0, 0, 0);
		cv::dilate(padded, padded, kernel);
		
		// extract contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(padded, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		for (int i = 0; i < hierarchy.size(); i++) {
			if (hierarchy[i][3] != -1) continue;
			if (contours[i].size() < 3) continue;

			Polygon polygon;
			if (add_right_corner) {
				Ring contour = addCornerToOpenCVContour(contours[i], padded);
				polygon.contour.resize(contour.size());
				for (int j = 0; j < contour.size(); j++) {
					polygon.contour[j] = cv::Point2f(std::round(contour[j].x * 0.25), std::round(contour[j].y * 0.25));
				}
			}
			else {
				polygon.contour.resize(contours[i].size());
				for (int j = 0; j < contours[i].size(); j++) {
					polygon.contour[j] = cv::Point2f(std::round(contours[i][j].x * 0.25), std::round(contours[i][j].y * 0.25));
				}
			}
			polygon.contour = removeRedundantPoint(polygon.contour);
			
			if (polygon.contour.size() >= 3) {
				// obtain all the holes inside this contour
				int hole_id = hierarchy[i][2];
				while (hole_id != -1) {
					Ring hole;
					if (add_right_corner) {
						hole = addCornerToOpenCVContour(contours[hole_id], padded);
						for (int j = 0; j < hole.size(); j++) {
							hole[j] = cv::Point2f(std::round(hole[j].x * 0.25), std::round(hole[j].y * 0.25));
						}
					}
					else {
						hole.resize(contours[hole_id].size());
						for (int j = 0; j < contours[hole_id].size(); j++) {
							hole[j] = cv::Point2f(std::round(contours[hole_id][j].x * 0.25), std::round(contours[hole_id][j].y * 0.25));
						}
					}
					hole = removeRedundantPoint(hole);
					polygon.holes.push_back(hole);
					hole_id = hierarchy[hole_id][0];
				}

				ans.push_back(polygon);
			}
		}

		return ans;
	}

	/**
	* Find the contour polygons from the image.
	*/
	std::vector<Polygon> findContours(const cv::Mat_<uchar>& image, int threshold, bool simplify, bool allow_diagonal, bool dilate) {
		std::vector<Polygon> polygons;

		cv::Mat mat = image.clone();
		cv::threshold(mat, mat, threshold, 255, cv::THRESH_BINARY);

		cv::Mat_<uchar> mat2 = mat.clone();

		// if diagonal is not allowwd, dilate the diagonal connection.
		if (!allow_diagonal) {
			while (true) {
				bool updated = false;
				for (int r = 0; r < mat.rows - 1; r++) {
					for (int c = 0; c < mat.cols - 1; c++) {
						if (mat2(r, c) == 255 && mat2(r + 1, c + 1) == 255 && mat2(r + 1, c) == 0 && mat2(r, c + 1) == 0) {
							updated = true;
							mat2(r + 1, c) = 255;
						}
						else if (mat2(r, c) == 0 && mat2(r + 1, c + 1) == 0 && mat2(r + 1, c) == 255 && mat2(r, c + 1) == 255) {
							updated = true;
							mat2(r, c) = 255;
						}
					}
				}
				if (!updated) break;
			}
		}

		// dilate the image
		if (dilate) {
			// resize x4
			cv::Mat_<uchar> img3;
			cv::resize(mat2, img3, cv::Size(mat2.cols * 4, mat2.rows * 4), 0, 0, cv::INTER_NEAREST);

			// add padding
			cv::Mat_<uchar> padded = cv::Mat_<uchar>::zeros(img3.rows + 1, img3.cols + 1);
			img3.copyTo(padded(cv::Rect(0, 0, img3.cols, img3.rows)));

			// dilate image
			cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 1, 1, 0, 1, 1, 0, 0, 0, 0);
			cv::dilate(padded, mat2, kernel);
		}

		// extract contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		if (simplify) {
			cv::findContours(mat2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		}
		else {
			cv::findContours(mat2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
		}

		for (int i = 0; i < hierarchy.size(); i++) {
			if (hierarchy[i][3] != -1) continue;
			if (contours[i].size() < 3) continue;

			util::Polygon polygon;
			polygon.contour.resize(contours[i].size());
			for (int j = 0; j < contours[i].size(); j++) {
				if (dilate) {
					polygon.contour[j] = cv::Point2f(std::round(contours[i][j].x * 0.25), std::round(contours[i][j].y * 0.25));
				}
				else {
					polygon.contour[j] = cv::Point2f(contours[i][j].x, contours[i][j].y);
				}
			}
			if (dilate) {
				polygon.contour = removeRedundantPoint(polygon.contour);
			}

			if (polygon.contour.size() >= 3) {
				// obtain all the holes inside this contour
				int hole_id = hierarchy[i][2];
				while (hole_id != -1) {
					// ignore holes in this implementation for simplicity
					util::Ring hole;
					hole.resize(contours[hole_id].size());
					for (int j = 0; j < contours[hole_id].size(); j++) {
						if (dilate) {
							hole[j] = cv::Point2f(std::round(contours[hole_id][j].x * 0.25), std::round(contours[hole_id][j].y * 0.25));
						}
						else {
							hole[j] = cv::Point2f(contours[hole_id][j].x, contours[hole_id][j].y);
						}
					}
					if (dilate)
						hole = removeRedundantPoint(hole);
					polygon.holes.push_back(hole);
					hole_id = hierarchy[hole_id][0];
				}

				polygons.push_back(polygon);
			}
		}

		return polygons;

	}

	Ring addCornerToOpenCVContour(const std::vector<cv::Point>& polygon, const cv::Mat_<uchar>& img) {
		Ring ans;
		
		for (int i = 0; i < polygon.size(); i++) {
			ans.push_back(polygon[i]);

			int next = (i + 1) % polygon.size();

			// check if the edge is diagonal
			if (std::abs(polygon[next].x - polygon[i].x) == 1 && std::abs(polygon[next].y - polygon[i].y) == 1) {
				// add a right angle corner
				cv::Point2f p(polygon[next].x, polygon[i].y);
				if (img.at<uchar>(p.y, p.x) == 255) ans.push_back(p);
				else {
					cv::Point2f p(polygon[i].x, polygon[next].y);
					if (img.at<uchar>(p.y, p.x) == 255) ans.push_back(p);
					else {
						cv::Point2f p(std::max(polygon[i].x, polygon[next].x), std::min(polygon[i].y, polygon[next].y));
						if ((p.x != polygon[i].x || p.y != polygon[i].y) && (p.x != polygon[next].x || p.y != polygon[next].y)) {
							ans.push_back(p);
						}
						else {
							cv::Point2f p(std::min(polygon[i].x, polygon[next].x), std::min(polygon[i].y, polygon[next].y));
							ans.push_back(p);
						}
					}
				}
			}
		}

		return removeRedundantPoint(ans);
	}

	/**
 	 * My custom contour extraction
	 * Currently, only the right-angle corner is supported, and only one contour is supported.
	 *
	 *
	 * @param img		input single-channel image (0 - background, 255 - footprint)
	 * @param contour	output contour polygon
	 */
	void findContour(const cv::Mat_<uchar>& img, std::vector<cv::Point>& contour) {
		contour.clear();

		// find the start point
		cv::Point start;
		bool found = false;
		for (int r = 0; r < img.rows && !found; r++) {
			for (int c = 0; c < img.cols; c++) {
				if (img.at<uchar>(r, c) == 255) {
					start = cv::Point(c, r);
					found = true;
					break;
				}
			}
		}

		cv::Point prev_dir(1, 0);
		cv::Point cur = start;
		contour.push_back(cur);
		int cnt = 0;
		do {
			cnt++;
			if (cnt > 1000) {
				break;
			}

			cv::Point left_dir(prev_dir.y, -prev_dir.x);
			cv::Point next = cur + left_dir;
			if (img.at<uchar>(next.y, next.x) == 255) {
				if (contour.size() > 0 && contour.back() != cur) contour.push_back(cur);
				cur = next;
				prev_dir = left_dir;
				continue;
			}

			/*
			cv::Point left_fore_dir = prev_dir + left_dir;
			if (std::abs(left_fore_dir.x) > 1) left_fore_dir.x /= std::abs(left_fore_dir.x);
			if (std::abs(left_fore_dir.y) > 1) left_fore_dir.y /= std::abs(left_fore_dir.y);
			next = cur + left_fore_dir;
			if (img.at<uchar>(next.y, next.x) == 255) {
			if (contour.size() > 0 && contour.back() != cur) contour.push_back(cur);
			cur = next;
			prev_dir = left_fore_dir;
			continue;
			}
			*/

			next = cur + prev_dir;
			if (img.at<uchar>(next.y, next.x) == 255) {
				cur = next;
				continue;
			}

			/*
			cv::Point right_fore_dir = prev_dir - left_dir;
			if (std::abs(right_fore_dir.x) > 1) right_fore_dir.x /= std::abs(right_fore_dir.x);
			if (std::abs(right_fore_dir.y) > 1) right_fore_dir.y /= std::abs(right_fore_dir.y);
			next = cur + right_fore_dir;
			if (img.at<uchar>(next.y, next.x) == 255) {
			if (contour.size() > 0 && contour.back() != cur) contour.push_back(cur);
			cur = next;
			prev_dir = right_fore_dir;
			continue;
			}
			*/

			cv::Point right_dir(-prev_dir.y, prev_dir.x);
			next = cur + right_dir;
			if (img.at<uchar>(next.y, next.x) == 255) {
				if (contour.size() > 0 && contour.back() != cur) contour.push_back(cur);
				cur = next;
				prev_dir = right_dir;
				continue;
			}

			cv::Point back_dir = -prev_dir;
			next = cur + back_dir;
			if (img.at<uchar>(next.y, next.x) == 255) {
				//contour.push_back(cur);
				cur = next;
				prev_dir = back_dir;
				continue;
			}

			break;
		} while (cur != start);
	}

	/**
	 * Create image from the contour.
	 */
	void createImageFromContour(int width, int height, const std::vector<cv::Point>& contour, const cv::Point& offset, cv::Mat_<uchar>& result, bool erode) {
		result = cv::Mat_<uchar>::zeros(height * 2, width * 2);
		std::vector<std::vector<cv::Point>> contour_points(1);

		contour_points[0].resize(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			contour_points[0][i] = cv::Point((contour[i].x + offset.x) * 2, (contour[i].y + offset.y) * 2);
		}

		cv::fillPoly(result, contour_points, cv::Scalar(255), cv::LINE_4);

		// erode image
		if (erode) {
			cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 0, 1, 0, 1, 1);
			cv::erode(result, result, kernel);
		}

		cv::resize(result, result, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
	}

	void createImageFromPolygon(int width, int height, const Polygon& polygon, const cv::Point& offset, cv::Mat_<uchar>& result) {
		result = cv::Mat_<uchar>::zeros(height * 2, width * 2);
		std::vector<std::vector<cv::Point>> contour_points(1 + polygon.holes.size());

		contour_points[0].resize(polygon.contour.size());
		Ring ring = polygon.contour.getActualPoints();
		for (int i = 0; i < ring.size(); i++) {
			contour_points[0][i] = cv::Point((ring[i].x + offset.x) * 2, (ring[i].y + offset.y) * 2);
		}
		for (int i = 0; i < polygon.holes.size(); i++) {
			contour_points[i + 1].resize(polygon.holes[i].size());
			Ring hole = polygon.holes[i].getActualPoints();
			for (int j = 0; j < hole.size(); j++) {
				contour_points[i + 1][j] = cv::Point((hole[j].x + offset.x) * 2, (hole[j].y + offset.y) * 2);
			}
		}
		cv::fillPoly(result, contour_points, cv::Scalar(255), cv::LINE_4);

		// erode image
		cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 0, 1, 0, 1, 1);
		cv::erode(result, result, kernel);

		cv::resize(result, result, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
	}

	std::vector<util::Polygon> intersection(const util::Polygon& polygon1, const util::Polygon& polygon2) {
		if (!isSimple(polygon1) || !isSimple(polygon2)) throw "Input polygon is not simple.";

		// convert polygon1 to CGAL polygon
		CGAL::Polygon_2<Kernel> pgn1;
		std::vector<cv::Point2f> contour1 = polygon1.contour.getActualPoints().points;
		counterClockwise(contour1);
		for (int i = 0; i < contour1.size(); i++) {
			pgn1.push_back(Kernel::Point_2(contour1[i].x, contour1[i].y));
		}
		if (!pgn1.is_simple()) throw "Input polygon is not simple.";
		std::vector<CGAL::Polygon_2<Kernel>> pgn1_holes(polygon1.holes.size());
		for (int i = 0; i < polygon1.holes.size(); i++) {
			std::vector<cv::Point2f> hole = polygon1.holes[i].getActualPoints().points;
			clockwise(hole);
			for (int j = 0; j < hole.size(); j++) {
				pgn1_holes[i].push_back(Kernel::Point_2(hole[j].x, hole[j].y));
			}
		}
		CGAL::Polygon_with_holes_2<Kernel> ph1(pgn1, pgn1_holes.begin(), pgn1_holes.end());

		// convert polygon2 to CGAL polygon
		CGAL::Polygon_2<Kernel> pgn2;
		std::vector<cv::Point2f> contour2 = polygon2.contour.getActualPoints().points;
		counterClockwise(contour2);
		for (int i = 0; i < contour2.size(); i++) {
			pgn2.push_back(Kernel::Point_2(contour2[i].x, contour2[i].y));
		}
		if (!pgn2.is_simple()) throw "Input polygon is not simple.";
		std::vector<CGAL::Polygon_2<Kernel>> pgn2_holes(polygon2.holes.size());
		for (int i = 0; i < polygon2.holes.size(); i++) {
			std::vector<cv::Point2f> hole = polygon2.holes[i].getActualPoints().points;
			clockwise(hole);
			for (int j = 0; j < hole.size(); j++) {
				pgn2_holes[i].push_back(Kernel::Point_2(hole[j].x, hole[j].y));
			}
		}
		CGAL::Polygon_with_holes_2<Kernel> ph2(pgn2, pgn2_holes.begin(), pgn2_holes.end());

		// compute intersection
		std::list<CGAL::Polygon_with_holes_2<Kernel>> result;
		CGAL::intersection(ph1, ph2, std::back_inserter(result));

		// convert the CGAL resutls to the vector of polygon
		std::vector<util::Polygon> ans(result.size());
		int polygon_id = 0;
		for (auto rit = result.begin(); rit != result.end(); rit++, polygon_id++) {
			for (auto it = rit->outer_boundary().vertices_begin(); it != rit->outer_boundary().vertices_end(); it++) {
				ans[polygon_id].contour.push_back(cv::Point2f(CGAL::to_double(it->x()), CGAL::to_double(it->y())));
			}

			ans[polygon_id].holes.resize(rit->number_of_holes());
			int hole_id = 0;
			for (auto it = rit->holes_begin(); it != rit->holes_end(); it++, hole_id++) {
				for (auto it2 = it->vertices_begin(); it2 != it->vertices_end(); it2++) {
					ans[polygon_id].holes[hole_id].push_back(cv::Point2f(CGAL::to_double(it2->x()), CGAL::to_double(it2->y())));
				}
			}
		}

		// remove the vertex if the two adjacent segments are colinear
		for (int i = ans.size() - 1; i >= 0; i--) {
			for (int j = ans[i].contour.points.size() - 1; j >= 0; j--) {
				int prev = (j - 1 + ans[i].contour.points.size()) % ans[i].contour.points.size();
				int next = (j + 1) % ans[i].contour.points.size();
				if (std::abs(crossProduct(ans[i].contour.points[j] - ans[i].contour.points[prev], ans[i].contour.points[next] - ans[i].contour.points[j])) < 0.00001) {
					ans[i].contour.points.erase(ans[i].contour.points.begin() + j);
				}
			}
			if (ans[i].contour.size() == 0) {
				ans.erase(ans.begin() + i);
				continue;
			}

			for (int j = ans[i].holes.size() - 1; j >= 0; j--) {
				for (int k = ans[i].holes[j].points.size() - 1; k >= 0; k--) {
					int prev = (k - 1 + ans[i].holes[j].points.size()) % ans[i].holes[j].points.size();
					int next = (k + 1) % ans[i].holes[j].points.size();
					if (std::abs(crossProduct(ans[i].holes[j].points[k] - ans[i].holes[j].points[prev], ans[i].holes[j].points[next] - ans[i].holes[j].points[k])) < 0.00001) {
						ans[i].holes[j].points.erase(ans[i].holes[j].points.begin() + k);
					}
				}

				if (ans[i].holes[j].size() == 0) ans[i].holes.erase(ans[i].holes.begin() + j);
			}

			if (!isSimple(ans[i])) ans.erase(ans.begin() + i);
		}

		return ans;
	}

	/**
	 * Approximate a polygon using DP algorithm.
	 * This function uses the OpenCV DP function, but if the resultant polygon is self-intersecting,
	 * it resolves the self-intersecting using an image-based approach.
	 */
	void approxPolyDP(const std::vector<cv::Point2f>& input_polygon, std::vector<cv::Point2f>& output_polygon, double epsilon, bool closed) {
		cv::approxPolyDP(input_polygon, output_polygon, epsilon, true);
		
		// If the polygon is self-intersecting, resolve it.
		while (!util::isSimple(output_polygon)) {
			util::Ring ring = resolveSelfIntersection(output_polygon);
			cv::approxPolyDP(ring.points, output_polygon, 1, closed);

			if (output_polygon.size() < 3) break;
		}
	}

	/*
	void snapPolygon(const std::vector<cv::Point2f>& ref_polygon, std::vector<cv::Point2f>& polygon, float snap_vertex_threshold, float snap_edge_threshold) {
		for (int i = 0; i < polygon.size(); i++) {
			// find the closest point
			double min_length = std::numeric_limits<double>::max();
			int min_index = -1;
			for (int j = 0; j < ref_polygon.size(); j++) {
				double length = cv::norm(polygon[i] - ref_polygon[j]);
				if (length < min_length) {
					min_length = length;
					min_index = j;
				}
			}

			if (min_length < snap_vertex_threshold) {
				polygon[i] = ref_polygon[min_index];
			}
			else {
				min_length = std::numeric_limits<double>::max();
				cv::Point2f min_pt;
				// find the closest edge
				for (int j = 0; j < ref_polygon.size(); j++) {
					int next = (j + 1) % ref_polygon.size();
					cv::Point2f pt;
					double length = closestPoint(ref_polygon[j], ref_polygon[next], polygon[i], true, pt);
					if (length < min_length) {
						min_length = length;
						min_pt = pt;
					}
				}

				if (min_length < snap_edge_threshold) {
					polygon[i] = min_pt;
				}
			}
		}
	}
	*/

	void snapPolygon(const std::vector<util::Polygon>& ref_polygons, util::Polygon& polygon, float snapping_threshold) {
		util::Polygon ans;

		std::vector<cv::Point2f> contour = polygon.contour.getActualPoints().points;
		util::snapPolygon(ref_polygons, contour, snapping_threshold);
		ans.contour.points = contour;
		if (!isSimple(ans.contour)) return;
		
		for (int j = 0; j < polygon.holes.size(); j++) {
			std::vector<cv::Point2f> hole = polygon.holes[j].getActualPoints().points;
			util::snapPolygon(ref_polygons, hole, snapping_threshold);
			ans.holes.push_back(hole);
		}

		if (isSimple(ans)) {
			polygon = ans;
		}
	}
	
	void snapPolygon(const std::vector<util::Polygon>& ref_polygons, std::vector<cv::Point2f>& polygon, float snapping_threshold) {
		std::vector<cv::Point2f> orig_polygon = polygon;

		for (int i = 0; i < polygon.size(); i++) {
			int i2 = (i + 1) % polygon.size();
			float len = length(polygon[i] - polygon[i2]);

			// find the closest almost-colinear edge from the reference polygons
			float max_len_ratio = 0;
			cv::Point2f pt1, pt2;

			for (int j = 0; j < ref_polygons.size(); j++) {
				std::vector<cv::Point2f> contour = ref_polygons[j].contour.getActualPoints().points;

				for (int k = 0; k < contour.size(); k++) {
					int k2 = (k + 1) % contour.size();

					float dot_product = std::abs(dotProduct(contour[k2] - contour[k], polygon[i2] - polygon[i]) / length(contour[k2] - contour[k]) / length(polygon[i2] - polygon[i]));
					if (dot_product < 0.95) continue;

					float ref_len = length(contour[k] - contour[k2]);
					float len_ratio = ref_len / len;

					std::vector<float> dist(4);
					dist[0] = distance(contour[k], contour[k2], polygon[i], true);
					dist[1] = distance(contour[k], contour[k2], polygon[i2], true);
					dist[2] = distance(polygon[i], polygon[i2], contour[k], true);
					dist[3] = distance(polygon[i], polygon[i2], contour[k2], true);

					for (int l = 0; l < dist.size(); l++) {
						for (int m = l + 1; m < dist.size(); m++) {
							if (dist[l] < snapping_threshold && dist[m] < snapping_threshold && len_ratio > 0.1 && len_ratio > max_len_ratio) {
								max_len_ratio = len_ratio;
								pt1 = contour[k];
								pt2 = contour[k2];
							}
						}
					}
				}

				for (int k = 0; k < ref_polygons[j].holes.size(); k++) {
					std::vector<cv::Point2f> hole = ref_polygons[j].holes[k].getActualPoints().points;
					for (int l = 0; l < hole.size(); l++) {
						int l2 = (l + 1) % hole.size();

						float dot_product = std::abs(dotProduct(hole[l2] - hole[l], polygon[i2] - polygon[i]) / length(hole[l2] - hole[l]) / length(polygon[i2] - polygon[i]));
						if (dot_product < 0.95) continue;

						float ref_len = length(hole[l] - hole[l2]);
						float len_ratio = ref_len / len;

						std::vector<float> dist(4);
						dist[0] = distance(hole[l], hole[l2], polygon[i], true);
						dist[1] = distance(hole[l], hole[l2], polygon[i2], true);
						dist[2] = distance(polygon[i], polygon[i2], hole[l], true);
						dist[3] = distance(polygon[i], polygon[i2], hole[l2], true);

						for (int m = 0; m < dist.size(); m++) {
							for (int n = m + 1; n < dist.size(); n++) {
								if (dist[m] < snapping_threshold && dist[n] < snapping_threshold && len_ratio > 0.1 && len_ratio > max_len_ratio) {
									max_len_ratio = len_ratio;
									pt1 = hole[l];
									pt2 = hole[l2];
								}
							}
						}
					}
				}
			}

			// snap the edge to the closest one
			if (max_len_ratio > 0) {
				snapEdge(pt1, pt2, polygon, i, i2);
			}
		}

		// remove the degenrated points
		for (int i = polygon.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + polygon.size()) % polygon.size();
			if (length(polygon[i] - polygon[prev]) < 0.001) polygon.erase(polygon.begin() + i);
		}

		/*
		if (!isSimple(polygon)) {
			polygon = orig_polygon;
		}
		*/
	}

	bool snapEdge(const cv::Point2f& p1, const cv::Point2f& p2, std::vector<cv::Point2f>& polygon, int i, int i2) {
		int prev = (i - 1 + polygon.size()) % polygon.size();
		int next = (i2 + 1) % polygon.size();

		double tab, tcd;
		cv::Point2f int_pt1;
		if (!segmentSegmentIntersection(p1, p2, polygon[i], polygon[prev], &tab, &tcd, false, int_pt1)) int_pt1 = polygon[i];
		cv::Point2f int_pt2;
		if (!segmentSegmentIntersection(p1, p2, polygon[i2], polygon[next], &tab, &tcd, false, int_pt2)) int_pt2 = polygon[i2];

		std::vector<cv::Point2f> orig_polygon = polygon;

		// snap the edge
		polygon[i] = int_pt1;
		polygon[i2] = int_pt2;

		if (!isSimple(polygon)) {
			polygon = orig_polygon;
			return false;
		}

		return true;
	}

	void snapPolygon2(const std::vector<util::Polygon>& ref_polygons, util::Polygon& polygon, float snapping_threshold) {
		util::Polygon ans;

		std::vector<cv::Point2f> contour = polygon.contour.getActualPoints().points;
		util::snapPolygon2(ref_polygons, contour, snapping_threshold);
		ans.contour.points = contour;
		if (!isSimple(ans.contour)) return;

		for (int j = 0; j < polygon.holes.size(); j++) {
			std::vector<cv::Point2f> hole = polygon.holes[j].getActualPoints().points;
			util::snapPolygon2(ref_polygons, hole, snapping_threshold);
			ans.holes.push_back(hole);
		}

		if (isSimple(ans)) {
			polygon = ans;
		}
	}

	void snapPolygon2(const std::vector<util::Polygon>& ref_polygons, std::vector<cv::Point2f>& polygon, float snapping_threshold) {
		std::vector<cv::Point2f> orig_polygon = polygon;

		for (int i = 0; i < polygon.size(); i++) {
			int i2 = (i + 1) % polygon.size();
			float len = length(polygon[i] - polygon[i2]);

			// find the closest almost-colinear edge from the reference polygons
			float max_len_ratio = 0;
			cv::Point2f pt1, pt2;

			for (int j = 0; j < ref_polygons.size(); j++) {
				std::vector<cv::Point2f> contour = ref_polygons[j].contour.getActualPoints().points;

				for (int k = 0; k < contour.size(); k++) {
					int k2 = (k + 1) % contour.size();

					float dot_product = std::abs(dotProduct(contour[k2] - contour[k], polygon[i2] - polygon[i]) / length(contour[k2] - contour[k]) / length(polygon[i2] - polygon[i]));
					if (dot_product < 0.98) continue;

					float ref_len = length(contour[k] - contour[k2]);
					float len_ratio = ref_len / len;

					std::vector<float> dist(4);
					dist[0] = distance(contour[k], contour[k2], polygon[i], true);
					dist[1] = distance(contour[k], contour[k2], polygon[i2], true);
					dist[2] = distance(polygon[i], polygon[i2], contour[k], true);
					dist[3] = distance(polygon[i], polygon[i2], contour[k2], true);

					for (int l = 0; l < dist.size(); l++) {
						for (int m = l + 1; m < dist.size(); m++) {
							if (dist[l] < snapping_threshold && dist[m] < snapping_threshold && len_ratio > 0.1 && len_ratio > max_len_ratio) {
								max_len_ratio = len_ratio;
								pt1 = contour[k];
								pt2 = contour[k2];
							}
						}
					}
				}

				for (int k = 0; k < ref_polygons[j].holes.size(); k++) {
					std::vector<cv::Point2f> hole = ref_polygons[j].holes[k].getActualPoints().points;
					for (int l = 0; l < hole.size(); l++) {
						int l2 = (l + 1) % hole.size();

						float dot_product = std::abs(dotProduct(hole[l2] - hole[l], polygon[i2] - polygon[i]) / length(hole[l2] - hole[l]) / length(polygon[i2] - polygon[i]));
						if (dot_product < 0.98) continue;

						float ref_len = length(hole[l] - hole[l2]);
						float len_ratio = ref_len / len;

						std::vector<float> dist(4);
						dist[0] = distance(hole[l], hole[l2], polygon[i], true);
						dist[1] = distance(hole[l], hole[l2], polygon[i2], true);
						dist[2] = distance(polygon[i], polygon[i2], hole[l], true);
						dist[3] = distance(polygon[i], polygon[i2], hole[l2], true);

						for (int m = 0; m < dist.size(); m++) {
							for (int n = m + 1; n < dist.size(); n++) {
								if (dist[m] < snapping_threshold && dist[n] < snapping_threshold && len_ratio > 0.1 && len_ratio > max_len_ratio) {
									max_len_ratio = len_ratio;
									pt1 = hole[l];
									pt2 = hole[l2];
								}
							}
						}
					}
				}
			}

			// snap the edge to the closest one
			if (max_len_ratio > 0) {
				snapEdge2(pt1, pt2, polygon, i, i2);
			}
		}

		// remove the degenrated points
		for (int i = polygon.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + polygon.size()) % polygon.size();
			if (length(polygon[i] - polygon[prev]) < 0.001) polygon.erase(polygon.begin() + i);
		}

		/*
		if (!isSimple(polygon)) {
		polygon = orig_polygon;
		}
		*/
	}

	void snapPolygon3(const std::vector<util::Polygon>& ref_polygons, util::Polygon& polygon, float snapping_threshold) {
		util::Polygon ans;

		std::vector<cv::Point2f> contour = polygon.contour.getActualPoints().points;
		util::snapPolygon3(ref_polygons, contour, snapping_threshold);
		ans.contour.points = contour;
		if (!isSimple(ans.contour)) return;

		for (int j = 0; j < polygon.holes.size(); j++) {
			std::vector<cv::Point2f> hole = polygon.holes[j].getActualPoints().points;
			util::snapPolygon3(ref_polygons, hole, snapping_threshold);
			ans.holes.push_back(hole);
		}

		if (isSimple(ans)) {
			polygon = ans;
		}
	}

	void snapPolygon3(const std::vector<util::Polygon>& ref_polygons, std::vector<cv::Point2f>& polygon, float snapping_threshold) {
		std::vector<cv::Point2f> orig_polygon = polygon;
		for (int i = 0; i < polygon.size(); i++) {
			float min_length = std::numeric_limits<float>::max();
			// find the closest point from the reference polygons
			cv::Point2f pt1;

			for (int j = 0; j < ref_polygons.size(); j++) {
				std::vector<cv::Point2f> contour = ref_polygons[j].contour.getActualPoints().points;

				for (int k = 0; k < contour.size(); k++) {
					float ref_len = length(contour[k] - polygon[i]);
					if (min_length > ref_len){
						pt1 = contour[k];
						min_length = ref_len;
					}

				}
			}
			// snap the edge to the closest one
			if (min_length < snapping_threshold) {
				polygon[i] = pt1;
			}
		}

		// remove the degenrated points
		for (int i = polygon.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + polygon.size()) % polygon.size();
			if (length(polygon[i] - polygon[prev]) < 0.001) polygon.erase(polygon.begin() + i);
		}
	}

	bool snapEdge2(const cv::Point2f& p1, const cv::Point2f& p2, std::vector<cv::Point2f>& polygon, int i, int i2) {
		int prev = (i - 1 + polygon.size()) % polygon.size();
		int next = (i2 + 1) % polygon.size();

		cv::Point2f int_pt1;
		closestPoint(p1, p2, polygon[i], false, int_pt1);
		cv::Point2f int_pt2;
		closestPoint(p1, p2, polygon[i2], false, int_pt2);

		std::vector<cv::Point2f> orig_polygon = polygon;

		// snap the edge
		polygon[i] = int_pt1;
		polygon[i2] = int_pt2;

		if (!isSimple(polygon)) {
			polygon = orig_polygon;
			return false;
		}

		return true;
	}

	float pointSegmentDistance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segment_only) {
		float r_numerator = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
		float r_denomenator = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);

		if (r_denomenator <= 0.0f) {
			return length(a, c);
		}

		float r = r_numerator / r_denomenator;

		if (segment_only && (r < 0 || r > 1)) {
			float dist1 = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);
			float dist2 = (c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y);
			if (dist1 < dist2) {
				return sqrt(dist1);
			}
			else {
				return sqrt(dist2);
			}
		}
		else {
			return abs((a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y)) / sqrt(r_denomenator);
		}
	}

	float length(const cv::Point2f& pt) {
		return std::sqrt(pt.x * pt.x + pt.y * pt.y);
	}

	float length(const cv::Point3f& pt) {
		return std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
	}

	float length(const cv::Point2f& p1, const cv::Point2f& p2) {
		return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	}

	float dotProduct(const cv::Point2f& v1, const cv::Point2f& v2) {
		return v1.x * v2.x + v1.y * v2.y;
	}

	float crossProduct(const cv::Point2f& v1, const cv::Point2f& v2) {
		return v1.x * v2.y - v1.y * v2.x;
	}

	cv::Point3f crossProduct(const cv::Point3f& v1, const cv::Point3f& v2) {
		cv::Point3f ans;
		ans.x = v1.y * v2.z - v1.z * v2.y;
		ans.y = v1.z * v2.x - v1.x * v2.z;
		ans.z = v1.x * v2.y - v1.y * v2.x;
		return ans;
	}

	/*
	 * Return the distance from segment ab to point c.
	 */
	float closestPoint(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segmentOnly, cv::Point2f& pt) {
		float r_numerator = dotProduct(c - a, b - a);
		float r_denomenator = dotProduct(b - a, b - a);

		if (r_denomenator <= 0.0f) {
			return cv::norm(a - c);
		}

		float r = r_numerator / r_denomenator;

		if (segmentOnly && (r < 0 || r > 1)) {
			float dist1 = std::hypot(c.x - a.x, c.y - a.y);
			float dist2 = std::hypot(c.x - b.x, c.y - b.y);
			if (dist1 < dist2) {
				pt = a;
				return dist1;
			}
			else {
				pt = b;
				return dist2;
			}
		}
		else {
			pt = a + (b - a) * r;
			return std::abs(crossProduct(c - a, b - a)) / sqrt(r_denomenator);
		}
	}

	/*
	* Return the distance from segment ab to point c.
	*/
	float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segmentOnly) {
		float r_numerator = dotProduct(c - a, b - a);
		float r_denomenator = dotProduct(b - a, b - a);

		if (r_denomenator <= 0.0f) {
			return cv::norm(a - c);
		}

		float r = r_numerator / r_denomenator;

		if (segmentOnly && (r < 0 || r > 1)) {
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
			return std::abs(crossProduct(c - a, b - a)) / sqrt(r_denomenator);
		}
	}

	float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d){
		float dis_a = distance(c, d, a, true);
		float dis_b = distance(c, d, b, true);
		float dis_c = distance(a, b, c, true);
		float dis_d = distance(a, b, d, true);
		float dis_1 = std::min(dis_a, dis_b);
		float dis_2 = std::min(dis_c, dis_d);
		return std::min(dis_1, dis_2);
	}

	bool segmentSegmentIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segmentOnly, cv::Point2f& intPoint) {
		cv::Point2f u = b - a;
		cv::Point2f v = d - c;

		if (length(u) < 0.0000001 || length(v) < 0.0000001) {
			return false;
		}

		double numer = v.x * (c.y - a.y) + v.y * (a.x - c.x);
		double denom = u.y * v.x - u.x * v.y;

		if (denom == 0.0)  {
			// they are parallel
			return false;
		}

		double t0 = numer / denom;

		cv::Point2f ipt = a + t0*u;
		cv::Point2f tmp = ipt - c;
		double t1;
		if (dotProduct(tmp, v) > 0.0) {
			t1 = length(tmp) / length(v);
		}
		else {
			t1 = -1.0 * length(tmp) / length(d, c);
		}

		//Check if intersection is within segments
		//if(segmentOnly && !( (t0 >= MTC_FLOAT_TOL) && (t0 <= 1.0-MTC_FLOAT_TOL) && (t1 >= MTC_FLOAT_TOL) && (t1 <= 1.0-MTC_FLOAT_TOL) ) ){
		if (segmentOnly && !((t0 >= 0.0000001) && (t0 <= 1.0 - 0.0000001) && (t1 >= 0.0000001) && (t1 <= 1.0 - 0.0000001))){
			return false;
		}

		*tab = t0;
		*tcd = t1;
		cv::Point2f dirVec = b - a;

		intPoint = a + t0 * dirVec;

		return true;
	}

	/**
	* Calculate the intersection of line a-b and line c-d.
	*
	* @param a				the first end point of the first line
	* @param b				the second end point of the first line
	* @param c				the first end point of the second line
	* @param d				the second end point of the second line
	* @param tab			the position of the intersecion on the first line (0 means point a, whereas 1 means point b).
	* @param tcd			the position of the intersecion on the second line (0 means point c, whereas 1 means point d).
	* @param segment_only	if true, find the intersection only on the line segments.
	* @param int_pt		intersecion point
	*/
	bool lineLineIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segment_only, cv::Point2f& int_pt) {
		cv::Point2f u = b - a;
		cv::Point2f v = d - c;

		if (cv::norm(u) < 0.0000001 || cv::norm(v) < 0.0000001) {
			return false;
		}

		double numer = v.x * (c.y - a.y) + v.y * (a.x - c.x);
		double denom = u.y * v.x - u.x * v.y;

		if (denom == 0.0)  {
			// they are parallel
			return false;
		}

		double t0 = numer / denom;

		cv::Point2f ipt = a + t0*u;
		cv::Point2f tmp = ipt - c;
		double t1;
		if (tmp.dot(v) > 0.0) {
			t1 = cv::norm(tmp) / cv::norm(v);
		}
		else {
			t1 = -1.0 * cv::norm(tmp) / cv::norm(d - c);
		}

		// check if intersection is within the segments
		if (segment_only && !((t0 >= 0.0000001) && (t0 <= 1.0 - 0.0000001) && (t1 >= 0.0000001) && (t1 <= 1.0 - 0.0000001))) {
			return false;
		}

		*tab = t0;
		*tcd = t1;
		cv::Point2f dirVec = b - a;

		int_pt = a + t0 * dirVec;

		return true;
	}


	/**
	* Estimate the orientation of the polygons using Hough Transform.
	*/
	float estimate(const std::vector<util::Polygon>& polygons) {
		int max_rho = 0;
		for (auto& polygon : polygons) {
			for (auto& pt : polygon.contour.points) {
				max_rho = std::max(max_rho, (int)std::ceil(pt.x * pt.x + pt.y * pt.y));
			}
		}

		cv::Mat_<float> HT(max_rho * 2 + 1, 180, 0.0f);
		for (auto& polygon : polygons) {
			for (auto& pt : polygon.contour.points) {
				for (int angle = 0; angle < 180; angle++) {
					float rho = pt.x * std::cos((float)angle / 180.0f * CV_PI) + pt.y * std::sin((float)angle / 180.0f * CV_PI);
					HT(std::round(rho) + max_rho, angle)++;
				}
			}
		}

		HT = HT.mul(HT);
		cv::Mat sumHT;
		cv::reduce(HT, sumHT, 0, cv::REDUCE_SUM);

		/*
		std::ofstream out("result.txt");
		for (int c = 0; c < sumHT.cols; c++) {
		out << sumHT.at<float>(0, c) << std::endl;
		}
		out.close();
		*/

		float max_votes = 0;
		int max_angle;
		for (int c = 0; c < sumHT.cols; c++) {
			if (sumHT.at<float>(0, c) > max_votes) {
				max_votes = sumHT.at<float>(0, c);
				max_angle = c;
			}
		}

		return max_angle / 180.0f * CV_PI;
	}

	/**
	* Estimate the orientation of the polygon using Hough Transform.
	*/
	float estimate(const std::vector<cv::Point2f>& polygon) {
		int max_rho = 0;
		for (auto& pt : polygon) {
			max_rho = std::max(max_rho, (int)std::ceil(pt.x * pt.x + pt.y * pt.y));
		}

		cv::Mat_<float> HT(max_rho * 2 + 1, 180, 0.0f);
		for (auto& pt : polygon) {
			for (int angle = 0; angle < 180; angle++) {
				float rho = pt.x * std::cos((float)angle / 180.0f * CV_PI) + pt.y * std::sin((float)angle / 180.0f * CV_PI);
				HT(std::round(rho) + max_rho, angle)++;
			}
		}

		HT = HT.mul(HT);
		cv::Mat sumHT;
		cv::reduce(HT, sumHT, 0, cv::REDUCE_SUM);

		/*
		std::ofstream out("result.txt");
		for (int c = 0; c < sumHT.cols; c++) {
		out << sumHT.at<float>(0, c) << std::endl;
		}
		out.close();
		*/

		float max_votes = 0;
		int max_angle;
		for (int c = 0; c < sumHT.cols; c++) {
			if (sumHT.at<float>(0, c) > max_votes) {
				max_votes = sumHT.at<float>(0, c);
				max_angle = c;
			}
		}

		return max_angle / 180.0f * CV_PI;
	}

	/**
	* calculate score for the polygon (average or sum) for only adjacent line segments.
	*/
	float calculateScoreRaOpt(const std::vector<cv::Point2f>& polygon, const std::vector<cv::Point2f>& init_polygon, int angle_threshold){
		float score = 0.0f;
		int total_segments = polygon.size();
		int valid_segments = 0;
		for (int i = 0; i < total_segments; i++){
			int first_start = i;
			int first_end = (i + 1) % total_segments;
			int second_start = (i + 1) % total_segments;
			int second_end = (i + 2) % total_segments;

			// init angles
			cv::Point2f a_init = cv::Point2f(init_polygon[first_start].x, init_polygon[first_start].y);
			cv::Point2f b_init = cv::Point2f(init_polygon[first_end].x, init_polygon[first_end].y);
			cv::Point2f c_init = cv::Point2f(init_polygon[second_start].x, init_polygon[second_start].y);;
			cv::Point2f d_init = cv::Point2f(init_polygon[second_end].x, init_polygon[second_end].y);
			float angle_init = lineLineAngle(a_init, b_init, c_init, d_init);
			// check
			cv::Point2f a = cv::Point2f(polygon[first_start].x, polygon[first_start].y);
			cv::Point2f b = cv::Point2f(polygon[first_end].x, polygon[first_end].y);
			cv::Point2f c = cv::Point2f(polygon[second_start].x, polygon[second_start].y);;
			cv::Point2f d = cv::Point2f(polygon[second_end].x, polygon[second_end].y);
			float angle = lineLineAngle(a, b, c, d);
			//std::cout << i << " " << angle <<std::endl;

			if (abs(angle - angle_init) > 2 * angle_threshold)
				return 0;

			if (abs(angle_init - 45) <= angle_threshold){

				if (abs(angle - angle_init) > angle_threshold)
					return 0;

				valid_segments++;
				// 45 - threshold ~ 45
				if (angle > 45 - angle_threshold && angle <= 45){
					score += (angle - 45 + angle_threshold) / angle_threshold;
				}
				// 45 ~ 45 + threshold
				else if (angle > 45 && angle <= 45 + angle_threshold){
					score += (45 + angle_threshold - angle) / angle_threshold;
				}
				// 135 - threshold ~ 135
				else if (angle > 135 - angle_threshold && angle <= 135){
					score += (angle - 135 + angle_threshold) / angle_threshold;
				}
				// 135 ~ 135 + threshold
				else if (angle > 135 && angle <= 135 + angle_threshold){
					score += (135 + angle_threshold - angle) / angle_threshold;
				}
				else
					score += 0;
			}
			else if (abs(angle_init - 90) <= angle_threshold){

				if (abs(angle - angle_init) > angle_threshold)
					return 0;

				valid_segments++;
				// 90 - threshold ~ 90
				if (angle > 90 - angle_threshold && angle <= 90){
					score += (angle - 90 + angle_threshold) / angle_threshold;
				}
				// 90 ~ 90 + threshold
				else if (angle > 90 && angle <= 90 + angle_threshold){
					score += (90 + angle_threshold - angle) / angle_threshold;
				}
				else
					score += 0;
			}
			else if (abs(angle_init - 135) <= angle_threshold){

				if (abs(angle - angle_init) > angle_threshold)
					return 0;

				valid_segments++;
				// 45 - threshold ~ 45
				if (angle > 45 - angle_threshold && angle <= 45){
					score += (angle - 45 + angle_threshold) / angle_threshold;
				}
				// 45 ~ 45 + threshold
				else if (angle > 45 && angle <= 45 + angle_threshold){
					score += (45 + angle_threshold - angle) / angle_threshold;
				}
				// 135 - threshold ~ 135
				else if (angle > 135 - angle_threshold && angle <= 135){
					score += (angle - 135 + angle_threshold) / angle_threshold;
				}
				// 135 ~ 135 + threshold
				else if (angle > 135 && angle <= 135 + angle_threshold){
					score += (135 + angle_threshold - angle) / angle_threshold;
				}
				else
					score += 0;
			}
			else{

			}
		}
		//std::cout << "score of polygon is " << score << std::endl;
		//std::cout << "-----------------" << std::endl;
		if (valid_segments == 0)
			return score;
		return score / valid_segments;
	}
	/**
	* calculate score for the polygon (average or sum) for any pair of line segments.
	*/
	//float calculateScoreParallelOpt(const std::vector<cv::Point2f>& polygon, const std::vector<cv::Point2f>& init_polygon, int angle_threshold){
	//	float score = 0.0f;
	//	int total_segments = polygon.size();
	//	int valid_segments = 0;
	//	int angle_index = 0;
	//	for (int i = 0; i < total_segments - 1; i++){
	//		for (int j = i + 1; j < total_segments; j++){
	//			int first_start = i;
	//			int first_end = (i + 1) % total_segments;
	//			int second_start = j;
	//			int second_end = (j + 1) % total_segments;

	//			// init angles
	//			cv::Point2f a_init = cv::Point2f(init_polygon[first_start].x, init_polygon[first_start].y);
	//			cv::Point2f b_init = cv::Point2f(init_polygon[first_end].x, init_polygon[first_end].y);
	//			cv::Point2f c_init = cv::Point2f(init_polygon[second_start].x, init_polygon[second_start].y);;
	//			cv::Point2f d_init = cv::Point2f(init_polygon[second_end].x, init_polygon[second_end].y);
	//			float angle_init = lineLineAngle(a_init, b_init, c_init, d_init);

	//			cv::Point2f a = cv::Point2f(polygon[first_start].x, polygon[first_start].y);
	//			cv::Point2f b = cv::Point2f(polygon[first_end].x, polygon[first_end].y);
	//			cv::Point2f c = cv::Point2f(polygon[second_start].x, polygon[second_start].y);;
	//			cv::Point2f d = cv::Point2f(polygon[second_end].x, polygon[second_end].y);
	//			float angle = lineLineAngle(a, b, c, d);

	//			if (abs(angle - angle_init) > 2 * angle_threshold)
	//				return 0;

	//			if (abs(angle_init) <= angle_threshold){
	//				if (abs(angle - angle_init) > angle_threshold)
	//					return 0;

	//				valid_segments++;
	//				// 0 ~ threshold
	//				if (angle >= 0 && angle <= angle_threshold){
	//					score += (angle_threshold - angle) / angle_threshold;
	//				}
	//				else
	//					score += 0;
	//			}
	//			else if (abs(angle_init - 180) <= angle_threshold){
	//				if (abs(angle - angle_init) > angle_threshold)
	//					return 0;

	//				valid_segments++;
	//				// 180 - threshold ~ 180
	//				if (angle > 180 - angle_threshold && angle <= 180){
	//					score += (angle - 180 + angle_threshold) / angle_threshold;
	//				}
	//				else
	//					score += 0;
	//			}
	//			else{

	//			}
	//			std::cout << angle_index++ << " angle is " << angle << std::endl;
	//		}
	//	}
	//	if (valid_segments == 0)
	//		return score;
	//	return score / valid_segments;
	//}

	/**
	* calculate score for the polygon (average or sum) for only adjacent line segments.
	*/
	float calculateScoreParallelOpt(const std::vector<cv::Point2f>& polygon, const std::vector<cv::Point2f>& init_polygon, int angle_threshold){
		float score = 0.0f;
		int total_segments = polygon.size();
		int valid_segments = 0;
		for (int i = 0; i < total_segments; i++){
			int first_start = i;
			int first_end = (i + 1) % total_segments;
			int second_start = (i + 1) % total_segments;
			int second_end = (i + 2) % total_segments;

			// init angles
			cv::Point2f a_init = cv::Point2f(init_polygon[first_start].x, init_polygon[first_start].y);
			cv::Point2f b_init = cv::Point2f(init_polygon[first_end].x, init_polygon[first_end].y);
			cv::Point2f c_init = cv::Point2f(init_polygon[second_start].x, init_polygon[second_start].y);;
			cv::Point2f d_init = cv::Point2f(init_polygon[second_end].x, init_polygon[second_end].y);
			float angle_init = lineLineAngle(a_init, b_init, c_init, d_init);
			// check
			cv::Point2f a = cv::Point2f(polygon[first_start].x, polygon[first_start].y);
			cv::Point2f b = cv::Point2f(polygon[first_end].x, polygon[first_end].y);
			cv::Point2f c = cv::Point2f(polygon[second_start].x, polygon[second_start].y);;
			cv::Point2f d = cv::Point2f(polygon[second_end].x, polygon[second_end].y);
			float angle = lineLineAngle(a, b, c, d);
			//std::cout << i << " " << angle <<std::endl;

			if (abs(angle - angle_init) > 2 * angle_threshold)
				return 0;

			if (abs(angle_init) <= angle_threshold){
				if (abs(angle - angle_init) > angle_threshold)
					return 0;

				valid_segments++;
				// 0 ~ threshold
				if (angle >= 0 && angle <= angle_threshold){
					score += (angle_threshold - angle) / angle_threshold;
				}
				else
					score += 0;
			}
			else if (abs(angle_init - 180) <= angle_threshold){
				if (abs(angle - angle_init) > angle_threshold)
					return 0;

				valid_segments++;
				// 180 - threshold ~ 180
				if (angle > 180 - angle_threshold && angle <= 180){
					score += (angle - 180 + angle_threshold) / angle_threshold;
				}
				else
					score += 0;
			}
			else{

			}
		}
		//std::cout << "score of polygon is " << score << std::endl;
		//std::cout << "-----------------" << std::endl;
		if (valid_segments == 0)
			return score;
		return score / valid_segments;
	}

	float lineLineAngle(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d){
		cv::Point2f vec1 = cv::Point2f(b.x - a.x, b.y - a.y);
		cv::Point2f vec2 = cv::Point2f(d.x - c.x, d.y - c.y);
		double norm_vec1 = cv::norm(vec1);
		double norm_vec2 = cv::norm(vec2);
		vec1.x = vec1.x / norm_vec1;
		vec1.y = vec1.y / norm_vec1;
		vec2.x = vec2.x / norm_vec2;
		vec2.y = vec2.y / norm_vec2;
		double dot = vec1.x * vec2.x + vec1.y * vec2.y;
		double ans = acos(dot);
		return ans / CV_PI * 180;
	}

	// define a score function based on the input angle of two lines
	float scoreFun(float angle, float threshold){
		float score = 0.0f;
		// 0 ~ threshold
		if (angle >= 0 && angle <= threshold)
			score = (threshold - angle) / threshold;
		// threshold ~ 45 - threshold
		else if (angle > threshold && angle <= 45 - threshold){
			score = 0;
		}
		// 45 - threshold ~ 45
		else if (angle > 45 - threshold && angle <= 45){
			score = (angle - 45 + threshold) / threshold;
		}
		// 45 ~ 45 + threshold
		else if (angle > 45 && angle <= 45 + threshold){
			score = (45 + threshold - angle) / threshold;
		}
		// 45 + threshold ~ 90 - threshold
		else if (angle > 45 + threshold && angle <= 90 - threshold){
			score = 0;
		}
		// 90 - threshold ~ 90
		else if (angle > 90 - threshold && angle <= 90){
			score = (angle - 90 + threshold) / threshold;
		}
		// 90 ~ 90 + threshold
		else if (angle > 90 && angle <= 90 + threshold){
			score = (90 + threshold - angle) / threshold;
		}
		// 90 + threshold ~ 135 - threshold
		else if (angle > 90 + threshold && angle <= 135 - threshold){
			score = 0;
		}
		//135 - threshold ~ 135
		else if (angle > 135 - threshold && angle <= 135){
			score = (angle - 135 + threshold) / threshold;
		}
		// 135 ~ 135 + threshold
		else if (angle > 135 && angle <= 135 + threshold){
			score = (135 + threshold - angle) / threshold;
		}
		// 135 + threshold ~ 180 - threshold
		else if (angle > 135 + threshold && angle <= 180 - threshold){
			score = 0;
		}
		// 180 - threshold ~ 180
		else if (angle > 180 - threshold && angle <= 180){
			score = (angle - 180 + threshold) / threshold;
		}
		else
			score = 0;
		return score;
	}

	cv::Point2f mirrorPoint(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c){
		cv::Point2f v = b - a;
		cv::Point2f n = cv::Point2f(v.y, -v.x);
		n = n / cv::norm(n);
		//std::cout << "normal vector is " << n <<std::endl;
		cv::Point2f mirror_point = n * ((a - c).dot(n)) * 2 + c;
		return mirror_point;
	}

	float calculateScorePointOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<cv::Point2f>& init_src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, const std::vector<std::vector<cv::Point2f>>& des_ini_layer_polygons, float dis_threshold){
		int valid_points = 0;
		float score = 0.0f;
		for (int k = 0; k < src_polygon.size(); k++){// current source polygon
			for (int i = 0; i < des_layer_polygons.size(); i++){// all polygons in the destination layer
				for (int j = 0; j < des_layer_polygons[i].size(); j++){// one polygon in the destination layer
					bool bValid = false;
					cv::Point2f src_init_p = init_src_polygon[k];
					cv::Point2f des_init_p = des_ini_layer_polygons[i][j];
					float dis_init = cv::norm(src_init_p - des_init_p);
					//std::cout << "dis_init from " << k << " to " << j << " is " << dis_init << std::endl;
					cv::Point2f src_p = src_polygon[k];
					cv::Point2f des_p = des_layer_polygons[i][j];
					if (dis_init <= dis_threshold){
						bValid = true;
					}
					if (bValid){
						float dis = cv::norm(src_p - des_p);
						if (dis >= dis_threshold)
							score += 0.0f;
						else
							score += (dis_threshold - dis) / dis_threshold;
						valid_points++;
					}
				}
			}
		}
		if (valid_points == 0){
			return score;
		}
		return score / valid_points;
	}

	float calculateScoreSegOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<cv::Point2f>& init_src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, const std::vector<std::vector<cv::Point2f>>& des_ini_layer_polygons, float dis_threshold, float angle_threshold){
		float score = 0.0f;
		int valid_segments = 0;
		int total_seg_src = src_polygon.size();
		for (int k = 0; k < total_seg_src; k++){// current source polygon
			for (int i = 0; i < des_layer_polygons.size(); i++){// all polygons in the destination layer
				int total_seg_des = des_layer_polygons[i].size();
				for (int j = 0; j < total_seg_des; j++){// one polygon in the destination layer
					bool bValid = false;
					cv::Point2f src_init_start = init_src_polygon[k];
					cv::Point2f src_init_end = init_src_polygon[(k + 1) % total_seg_src];
					cv::Point2f des_init_start = des_ini_layer_polygons[i][j];
					cv::Point2f des_init_end = des_ini_layer_polygons[i][(j + 1) % total_seg_des];
					// distance check
					float dis_init = distance(src_init_start, src_init_end, des_init_start, des_init_end);
					//std::cout << "dis_check ( " << k << " , " << (k + 1) % total_seg_src << ")  to (" << j << ", " << (j + 1) % total_seg_des << ") is "<< dis_init << std::endl;
					if (dis_init > dis_threshold){
						continue;
					}
					// angle check
					float angle_init = lineLineAngle(src_init_start, src_init_end, des_init_start, des_init_end);
					//std::cout << "angle_check ( " << k << " , " << (k + 1) % total_seg_src << ")  to (" << j << ", " << (j + 1) % total_seg_des << ") is " << angle_init << std::endl;
					if (!(abs(angle_init) <= angle_threshold || abs(180 - angle_init) <= angle_threshold)){
						//std::cout << "angle_check failed" << std::endl;
						continue;
					}
					// angle score
					cv::Point2f src_start = src_polygon[k];
					cv::Point2f src_end = src_polygon[(k + 1) % total_seg_src];
					cv::Point2f des_start = des_layer_polygons[i][j];
					cv::Point2f des_end = des_layer_polygons[i][(j + 1) % total_seg_des];
					float angle = lineLineAngle(src_start, src_end, des_start, des_end);
					angle = std::min(abs(angle), abs(180 - angle));
					if (angle >= angle_threshold)
						score += 0.0f;
					else
						score += 0.5 * (angle_threshold - angle) / angle_threshold;

					// distance score
					float dis = distance(src_start, src_end, des_start, des_end);
					if (dis >= dis_threshold)
						score += 0.0f;
					else
						score += 0.5 * (dis_threshold - dis) / dis_threshold;

					valid_segments++;
				}
			}
		}
		if (valid_segments == 0)
			return score;
		return score / valid_segments;
	}

	bool isTangent(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d) {
		cv::Point2f u = b - a;
		cv::Point2f v = d - c;

		if (length(u) < 0.0000001 || length(v) < 0.0000001) {
			return false;
		}

		if (std::abs(crossProduct(u, c - a)) > 0.0000001) return false;

		float ca = dotProduct(u, c - a) / length(u) / length(u);
		if (ca > 0 && ca < 1) return true;

		float da = dotProduct(u, d - a) / length(u) / length(u);
		if (da > 0 && da < 1) return true;

		float ac = dotProduct(v, a - c) / length(v) / length(v);
		if (ac > 0 && ac < 1) return true;

		float bc = dotProduct(v, b - c) / length(v) / length(v);
		if (bc > 0 && bc < 1) return true;

		return false;
	}

	/**
	 * Triangulate the polygon into triangles using the Delaunay triangulation.
	 * This function requires the polygon without any self-intersection. If the polygon has
	 * any self-intersection, this function will return no triangle.
	 *
	 * @param points		contour
	 * @return			list of triangles
	 */
	std::vector<std::vector<cv::Point2f>> tessellate(const Ring& points) {
		return tessellate(points, {});
	}

	/**
	 * Triangulate the polygon with holes into triangles using the Delaunay triangulation.
	 * This function requires the polygon without any self-intersection. If the polygon has
	 * any self-intersection, this function will return no triangle.
	 *
	 * @param points	contour
	 * @param holes		holes
	 * @return			list of triangles
	 */
	std::vector<std::vector<cv::Point2f>> tessellate(const Ring& points, const std::vector<Ring>& holes) {
		std::vector<std::vector<cv::Point2f>> ans;

		// Don't triangulate if there are only less than 3 points.
		if (points.size() == 0) {
			return{};
		}
		else if (points.size() <= 2) {
			return{ points.points };
		}

		// Insert the polygons into a constrained triangulation.
		CDT cdt;
		CGAL::Partition_traits_2<Kernel>::Polygon_2 polygon;
		for (int i = 0; i < points.size(); i++) {
			polygon.push_back(CDT::Point(points[i].x, points[i].y));
		}

		if (polygon.is_simple()) {
			cdt.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
			for (int i = 0; i < holes.size(); i++) {
				CGAL::Partition_traits_2<Kernel>::Polygon_2 polygon;
				for (int j = 0; j < holes[i].size(); j++) {
					polygon.push_back(CDT::Point(holes[i][j].x, holes[i][j].y));
				}
				cdt.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
			}

			//Mark facets that are inside the domain bounded by the polygon
			mark_domains(cdt);

			for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
				if (fit->info().in_domain()) {
					std::vector<cv::Point2f> pol;
					for (int i = 0; i < 3; i++) {
						CDT::Vertex_handle vh = fit->vertex(i);
						pol.push_back(cv::Point2f(vh->point().exact().x().to_double(), vh->point().exact().y().to_double()));
					}

					util::counterClockwise(pol);
					ans.push_back(pol);
				}
			}
		}

		return ans;
	}

	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border) {
		if (start->info().nesting_level != -1){
			return;
		}
		std::list<CDT::Face_handle> queue;
		queue.push_back(start);
		while (!queue.empty()){
			CDT::Face_handle fh = queue.front();
			queue.pop_front();
			if (fh->info().nesting_level == -1){
				fh->info().nesting_level = index;
				for (int i = 0; i < 3; i++){
					CDT::Edge e(fh, i);
					CDT::Face_handle n = fh->neighbor(i);
					if (n->info().nesting_level == -1){
						if (ct.is_constrained(e)) {
							border.push_back(e);


						}
						else queue.push_back(n);
					}
				}
			}
		}
	}

	//explore set of facets connected with non constrained edges,
	//and attribute to each such set a nesting level.
	//We start from facets incident to the infinite vertex, with a nesting
	//level of 0. Then we recursively consider the non-explored facets incident 
	//to constrained edges bounding the former set and increase the nesting level by 1.
	//Facets in the domain are those with an odd nesting level.
	void mark_domains(CDT& cdt) {
		for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
			it->info().nesting_level = -1;
		}
		std::list<CDT::Edge> border;
		mark_domains(cdt, cdt.infinite_face(), 0, border);
		while (!border.empty()){
			CDT::Edge e = border.front();
			border.pop_front();
			CDT::Face_handle n = e.first->neighbor(e.second);
			if (n->info().nesting_level == -1){
				mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
			}
		}
	}

}
