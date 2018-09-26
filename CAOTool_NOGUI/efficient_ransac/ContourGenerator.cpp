#include "ContourGenerator.h"
#include "CurveDetector.h"
#include "LineDetector.h"

ContourGenerator::ContourGenerator() {
}

ContourGenerator::~ContourGenerator() {
}

void ContourGenerator::generate(const util::Polygon& polygon, const std::vector<std::pair<int, std::shared_ptr<efficient_ransac::PrimitiveShape>>>& shapes, std::vector<cv::Point2f>& contour, std::vector<int>& contourPointsType, float max_error, float angle_threshold) {
	if (shapes.size() == 1){
		if (efficient_ransac::Circle* circle = dynamic_cast<efficient_ransac::Circle*>(shapes[0].second.get())) {
			int num = circle->angleRange() / CV_PI * 180 / 10;
			contour.push_back(circle->startPoint());
			for (int k = 1; k < num; k++) {
				float angle = circle->startAngle() + circle->signedAngleRange() / num * k;
				contour.push_back(circle->center() + circle->radius() * cv::Point2f(std::cos(angle), std::sin(angle)));
			}
			contour.push_back(circle->endPoint());
		}
		else{
			contour.clear();
		}
	}
	else if (shapes.size() == 2){
		if (efficient_ransac::Circle* circle = dynamic_cast<efficient_ransac::Circle*>(shapes[0].second.get())) {
			int num = circle->angleRange() / CV_PI * 180 / 10;
			contour.push_back(circle->startPoint());
			for (int k = 1; k < num; k++) {
				float angle = circle->startAngle() + circle->signedAngleRange() / num * k;
				contour.push_back(circle->center() + circle->radius() * cv::Point2f(std::cos(angle), std::sin(angle)));
			}
			contour.push_back(circle->endPoint());
			if (efficient_ransac::Circle* circle2 = dynamic_cast<efficient_ransac::Circle*>(shapes[1].second.get())) {
				int num = circle2->angleRange() / CV_PI * 180 / 10;
				contour.push_back(circle2->startPoint());
				for (int k = 1; k < num; k++) {
					float angle = circle2->startAngle() + circle2->signedAngleRange() / num * k;
					contour.push_back(circle2->center() + circle2->radius() * cv::Point2f(std::cos(angle), std::sin(angle)));
				}
				contour.push_back(circle2->endPoint());
			}
			else if (efficient_ransac::Line* line2 = dynamic_cast<efficient_ransac::Line*>(shapes[1].second.get())) {
				contour.push_back(line2->startPoint());
				contour.push_back(line2->endPoint());
			}
		}
		else if (efficient_ransac::Line* line = dynamic_cast<efficient_ransac::Line*>(shapes[0].second.get())) {
			contour.push_back(line->startPoint());
			contour.push_back(line->endPoint());
			if (efficient_ransac::Circle* circle2 = dynamic_cast<efficient_ransac::Circle*>(shapes[1].second.get())) {
				int num = circle2->angleRange() / CV_PI * 180 / 10;
				contour.push_back(circle2->startPoint());
				for (int k = 1; k < num; k++) {
					float angle = circle2->startAngle() + circle2->signedAngleRange() / num * k;
					contour.push_back(circle2->center() + circle2->radius() * cv::Point2f(std::cos(angle), std::sin(angle)));
				}
				contour.push_back(circle2->endPoint());
			}
			else if (efficient_ransac::Line* line2 = dynamic_cast<efficient_ransac::Line*>(shapes[1].second.get())) {
				contour.push_back(line2->startPoint());
				contour.push_back(line2->endPoint());
			}
		}

	}
	else{
		for (int j = 0; j < shapes.size(); j++) {
			if (efficient_ransac::Circle* circle = dynamic_cast<efficient_ransac::Circle*>(shapes[j].second.get())) {
				int num = circle->angleRange() / CV_PI * 180 / 10;
				contour.push_back(circle->startPoint());
				contourPointsType.push_back(1);
				for (int k = 1; k < num; k++) {
					float angle = circle->startAngle() + circle->signedAngleRange() / num * k;
					contour.push_back(circle->center() + circle->radius() * cv::Point2f(std::cos(angle), std::sin(angle)));
					contourPointsType.push_back(1);
				}
				contour.push_back(circle->endPoint());
				contourPointsType.push_back(1);
				int j2 = (j + 1) % shapes.size();
				// we assume the next shape is Line Segment
				contour.push_back(shapes[j2].second->startPoint());
				contourPointsType.push_back(0);
			}
			else if (efficient_ransac::Line* line = dynamic_cast<efficient_ransac::Line*>(shapes[j].second.get())) {
				int j2 = (j + 1) % shapes.size();
				if (efficient_ransac::Circle* circle2 = dynamic_cast<efficient_ransac::Circle*>(shapes[j2].second.get())) {
					contour.push_back(line->endPoint());
					contourPointsType.push_back(0);
					continue;
				}
				else if (efficient_ransac::Line* line2 = dynamic_cast<efficient_ransac::Line*>(shapes[j2].second.get())) {

					// check if two adjacent shapes are almost collinear
					cv::Point2f dir1 = shapes[j].second->endPoint() - shapes[j].second->startPoint();
					cv::Point2f dir2 = shapes[j2].second->endPoint() - shapes[j2].second->startPoint();
					if (std::abs(dir1.dot(dir2) / cv::norm(dir1) / cv::norm(dir2)) >= std::cos(angle_threshold)) {
						// if two shapes are close enough, merge them
						if (line->distance(line2->startPoint()) < max_error) continue;
						else {
							// simpliy connect two shapes
							contour.push_back(line->endPoint());
							contourPointsType.push_back(0);
							contour.push_back(line2->startPoint());
							contourPointsType.push_back(0);
							continue;
						}
					}

					// calculate the intersection between the current shape and the next one
					cv::Point2f int_pt;
					double tab, tcd;
					if (util::lineLineIntersection(shapes[j].second->startPoint(), shapes[j].second->endPoint(), shapes[j2].second->endPoint(), shapes[j2].second->startPoint(), &tab, &tcd, false, int_pt)) {
						bool valid = false;
						int start_index = shapes[j].second->startIndex();
						int end_index = shapes[j2].second->endIndex();
						if (start_index > end_index) end_index += polygon.contour.size();
						for (int k = start_index; k <= end_index; k++) {
							int index = (k + polygon.contour.size()) % polygon.contour.size();
							if (cv::norm(polygon.contour[index] - int_pt) < max_error) {
								valid = true;
								break;
							}
						}
						// we also need to make sure this intersection point is not inside of the line segment or in the reverse order
						float dis_start_j = cv::norm(shapes[j].second->startPoint() - int_pt);
						float dis_end_j = cv::norm(shapes[j].second->endPoint() - int_pt);
						float dis_j = cv::norm(shapes[j].second->startPoint() - shapes[j].second->endPoint());
						if (valid){
							if (dis_start_j < dis_end_j){
								valid = false;
							}
							/*else if (abs(dis_start_j + dis_end_j - dis_j) < 0.001){
							valid = false;
							}*/
							else{

							}
						}
						float dis_start_j2 = cv::norm(shapes[j2].second->startPoint() - int_pt);
						float dis_end_j2 = cv::norm(shapes[j2].second->endPoint() - int_pt);
						float dis_j2 = cv::norm(shapes[j2].second->startPoint() - shapes[j2].second->endPoint());
						if (valid){
							if (dis_start_j2 > dis_end_j2){
								valid = false;
							}
							/*else if (abs(dis_start_j2 + dis_end_j2 - dis_j2) < 0.001){
							valid = false;
							}*/
							else{

							}
						}
						if (valid) {
							contour.push_back(int_pt);
							contourPointsType.push_back(0);
							continue;
						}
					}

					// simpliy connect two shapes
					contour.push_back(line->endPoint());
					contourPointsType.push_back(0);
					contour.push_back(line2->startPoint());
					contourPointsType.push_back(0);

					//contour.push_back(line->startPoint());
					//contour.push_back(line->endPoint());
				}
			}
		}
	}

}