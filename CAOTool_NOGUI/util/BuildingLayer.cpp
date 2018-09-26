#include "BuildingLayer.h"

namespace util {

	BuildingLayer::BuildingLayer(int building_id, float bottom_height, float top_height) {
		this->building_id = building_id;
		this->bottom_height = bottom_height;
		this->top_height = top_height;
	}

	BuildingLayer::BuildingLayer(int building_id, const std::vector<util::Polygon>& footprints, float bottom_height, float top_height) {
		this->building_id = building_id;
		this->footprints = footprints;
		this->bottom_height = bottom_height;
		this->top_height = top_height;
	}

	std::vector<util::Polygon> BuildingLayer::selectRepresentativeContours() {
		// calculate the bounding box
		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		for (auto& polygons : raw_footprints) {
			for (auto& polygon : polygons) {
				Ring contour = polygon.contour.getActualPoints();
				for (auto& pt : contour) {
					min_x = std::min(min_x, (int)pt.x);
					min_y = std::min(min_y, (int)pt.y);
					max_x = std::max(max_x, (int)(pt.x + 0.5));
					max_y = std::max(max_y, (int)(pt.y + 0.5));
				}
			}
		}

		// calculate the average contour image
		int width = max_x - min_x + 1;
		int height = max_y - min_y + 1;
		cv::Mat_<double> total_img = cv::Mat_<double>::zeros(height * 2, width * 2);
		for (auto& polygons : raw_footprints) {
			cv::Mat_<uchar> img = cv::Mat_<uchar>::zeros(height * 2, width * 2);
			for (auto& polygon : polygons) {
				std::vector<std::vector<cv::Point>> contour_points(1 + polygon.holes.size());
				Ring contour = polygon.contour.getActualPoints();
				contour_points[0].resize(contour.size());
				for (int i = 0; i < contour.size(); i++) {
					contour_points[0][i] = cv::Point((contour[i].x - min_x) * 2, (contour[i].y - min_y) * 2);
				}
				for (int i = 0; i < polygon.holes.size(); i++) {
					Ring hole = polygon.holes[i].getActualPoints();
					contour_points[i + 1].resize(hole.size());
					for (int j = 0; j < hole.size(); j++) {
						contour_points[i + 1][j] = cv::Point((hole[j].x - min_x) * 2, (hole[j].y - min_y) * 2);
					}
				}
				cv::fillPoly(img, contour_points, cv::Scalar(255), cv::LINE_4);
			}

			cv::Mat_<double> img_double;
			img.convertTo(img_double, CV_64F);
			total_img += img_double;
		}
		total_img /= raw_footprints.size();
		cv::Mat_<uchar> mean_img;
		total_img.convertTo(mean_img, CV_8U);

		cv::Mat_<uchar> mean_img_thresholded;
		cv::threshold(mean_img, mean_img_thresholded, 80, 255, cv::THRESH_BINARY);

		// erode image
		cv::Mat_<uchar> eroded_img;
		cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 0, 1, 0, 1, 1);
		cv::erode(mean_img_thresholded, eroded_img, kernel);

		cv::resize(eroded_img, mean_img_thresholded, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);

		// extract contours
		std::vector<util::Polygon> contours = util::findContours(mean_img_thresholded, false);
		for (int i = 0; i < contours.size(); i++) {
			contours[i].translate(min_x, min_y);
		}

		this->footprints = contours;
		this->presentativeContours = contours;

		return contours;
	}

	int BuildingLayer::getTopHeight() {
		int ans = top_height;
		for (auto& child : children) {
			ans = std::max(ans, child->getTopHeight());
		}
		return ans;
	}

}