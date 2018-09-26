#include "EfficientRansacCurveDetector.h"
#include "PointSetShapeDetection.h"

namespace util {

	bool EfficientRansacCurveDetector::detect(const util::Polygon& polygon) {
		// calculate bounding box
		cv::Rect bbox = boundingBox(polygon.contour.getActualPoints().points);

		std::vector<std::pair<cv::Point3f, cv::Point3f>> point_cloud;

		cv::Mat_<uchar> image;
		createImageFromPolygon(bbox.width, bbox.height, polygon, cv::Point(-bbox.x, -bbox.y), image);

		// extract contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

		for (int i = 0; i < hierarchy.size(); i++) {
			if (hierarchy[i][3] != -1) continue;
			if (contours[i].size() < 3) continue;

			for (int j = 0; j < contours[i].size(); j++) {
				int prev = (j - 1 + contours[i].size()) % contours[i].size();
				int next = (j + 1) % contours[i].size();
				cv::Point2f dir(contours[i][next].x - contours[i][prev].x, contours[i][next].y - contours[i][prev].y);
				cv::Point2f normal(dir.y, -dir.x);

				point_cloud.push_back({ cv::Point3f(contours[i][j].x, contours[i][j].y, 0), cv::Point3f(normal.x, normal.y, 0) });
			}

			// obtain all the holes inside this contour
			int hole_id = hierarchy[i][2];
			while (hole_id != -1) {
				Ring hole;
				for (int j = 0; j < contours[hole_id].size(); j++) {
					int prev = (j - 1 + contours[hole_id].size()) % contours[hole_id].size();
					int next = (j + 1) % contours[hole_id].size();
					cv::Point2f dir(contours[hole_id][next].x - contours[hole_id][prev].x, contours[hole_id][next].y - contours[hole_id][prev].y);
					cv::Point2f normal(dir.y, -dir.x);

					point_cloud.push_back({ cv::Point3f(contours[hole_id][j].x, contours[hole_id][j].y, 0), cv::Point3f(normal.x, normal.y, 0) });
				}

				hole_id = hierarchy[hole_id][0];
			}
		}

		for (int iter = 0; iter < 20; iter++) {
			//if (pointcloud::shape::detectCurve(point_cloud, 0.000000001, std::max((int)(point_cloud.size() * 0.18), 10000), 5, 10, 0.9, 150, 400)) return true;
			if (pointcloud::shape::detectCurve(point_cloud, 0.000000001, std::max((int)(point_cloud.size() * 0.09), std::min((int)point_cloud.size(), 200)), 5, 10, 0.9, 150, 400)) return true;
		}
		return false;
	}

	bool EfficientRansacCurveDetector::detect2(const std::vector<std::vector<util::Polygon>>& polygons) {
		// calculate bounding box
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();;
		int max_y = -std::numeric_limits<int>::max();
		for (int z = 0; z < polygons.size(); z++) {
			for (int i = 0; i < polygons[z].size(); i++) {
				cv::Rect bbox = boundingBox(polygons[z][i].contour.getActualPoints().points);
				min_x = std::min(min_x, bbox.x);
				max_x = std::max(max_x, bbox.x + bbox.width);
				min_y = std::min(min_y, bbox.y);
				max_y = std::max(max_y, bbox.y + bbox.height);
			}
		}

		std::vector<std::pair<cv::Point3f, cv::Point3f>> point_cloud;

		for (int z = 0; z < polygons.size(); z++) {
			for (int pi = 0; pi < polygons[z].size(); pi++) {
				cv::Mat_<uchar> image;
				createImageFromPolygon(max_x - min_x, max_y - min_y, polygons[z][pi], cv::Point(-min_x, -min_y), image);

				// extract contours
				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

				for (int i = 0; i < hierarchy.size(); i++) {
					if (hierarchy[i][3] != -1) continue;
					if (contours[i].size() < 3) continue;

					for (int j = 0; j < contours[i].size(); j++) {
						int prev = (j - 1 + contours[i].size()) % contours[i].size();
						int next = (j + 1) % contours[i].size();
						cv::Point2f dir(contours[i][next].x - contours[i][prev].x, contours[i][next].y - contours[i][prev].y);
						cv::Point2f normal(dir.y, -dir.x);

						point_cloud.push_back({ cv::Point3f(contours[i][j].x, contours[i][j].y, z), cv::Point3f(normal.x, normal.y, 0) });
					}

					// obtain all the holes inside this contour
					int hole_id = hierarchy[i][2];
					while (hole_id != -1) {
						Ring hole;
						for (int j = 0; j < contours[hole_id].size(); j++) {
							int prev = (j - 1 + contours[hole_id].size()) % contours[hole_id].size();
							int next = (j + 1) % contours[hole_id].size();
							cv::Point2f dir(contours[hole_id][next].x - contours[hole_id][prev].x, contours[hole_id][next].y - contours[hole_id][prev].y);
							cv::Point2f normal(dir.y, -dir.x);

							point_cloud.push_back({ cv::Point3f(contours[hole_id][j].x, contours[hole_id][j].y, z), cv::Point3f(normal.x, normal.y, 0) });
						}

						hole_id = hierarchy[hole_id][0];
					}
				}
			}
		}

		for (int iter = 0; iter < 10; iter++) {
			if (pointcloud::shape::detectCurve(point_cloud, 0.000000001, std::max((int)(point_cloud.size() * 0.11), std::min((int)point_cloud.size(), 8000)), 5, 10, 0.9, 150, 400)) return true;
			//if (pointcloud::shape::detectCurve(point_cloud, 0.000000001, std::max((int)(point_cloud.size() * 0.18), std::min((int)point_cloud.size(), 10000)), 10, 10, 0.9, 150, 400)) return true;
		}
		return false;
	}
	
}