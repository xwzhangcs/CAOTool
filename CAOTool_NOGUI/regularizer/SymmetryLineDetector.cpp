#include "SymmetryLineDetector.h"

SymmetryLineDetector::SymmetryLineDetector() {
}

SymmetryLineDetector::~SymmetryLineDetector() {
}

std::vector<cv::Point2f> SymmetryLineDetector::fitSymmetryLine(const std::vector<cv::Point2f>& polygon) {
	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	for (int i = 0; i < polygon.size(); i++) {
		min_x = std::min(min_x, polygon[i].x);
		min_y = std::min(min_y, polygon[i].y);
		max_x = std::max(max_x, polygon[i].x);
		max_y = std::max(max_y, polygon[i].y);
	}
	int max_unit = std::max(max_x - min_x, max_y - min_y);
	std::vector<cv::Point2f> normalized_polygon(polygon.size());
	for (int i = 0; i < polygon.size(); i++) {
		normalized_polygon[i] = cv::Point2f((polygon[i].x - min_x) / max_unit, (polygon[i].y - min_y) / max_unit);
	}

	try {

		column_vector starting_point(3);
		starting_point(0) = 0;
		starting_point(1) = 0;
		starting_point(2) = 0;
		for (int i = 0; i < normalized_polygon.size(); i++){
			starting_point(0) += normalized_polygon[i].x;
			starting_point(1) += normalized_polygon[i].y;
		}
		starting_point(0) = starting_point(0) / normalized_polygon.size();
		starting_point(1) = starting_point(1) / normalized_polygon.size();
		starting_point(2) = 0.5 * CV_PI;
		
		BFGSSolver solver(normalized_polygon);
		find_max_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-6), solver, starting_point, 1, 0.0001);
		//find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), solver, starting_point, 0, 0.00001);
		std::vector<cv::Point2f> ans(2);
		//ans[0].x = starting_point(0) * max_unit + min_x;
		//ans[0].y = starting_point(1) * max_unit + min_y;	
		
		int ratio = 0;
		for (int i = 1; i <= 10; i++){
			if (starting_point(0) - 0.1 * i * cos(starting_point(2)) > 0.9 || starting_point(0) - 0.1 * i * cos(starting_point(2)) < 0.1){
				ratio = i;
				break;
			}
		}
		if (ratio == 0)
			ratio = 5;
		ans[0].x = (starting_point(0) - 0.1 * ratio * cos(starting_point(2)))* max_unit + min_x;
		ans[0].y = (starting_point(1) - 0.1 * ratio * sin(starting_point(2)))* max_unit + min_y;
		//std::cout << "ans[0] is " << ans[0].x << "," << ans[0].y << std::endl;
		ratio = 0;
		for (int i = 1; i <= 10; i++){
			if (starting_point(0) + 0.1 * i * cos(starting_point(2)) > 0.9 || starting_point(0) - 0.1 * i * cos(starting_point(2)) < 0.1){
				ratio = i;
				break;
			}
		}
		if (ratio == 0)
			ratio = 5;
		ans[1].x = (starting_point(0) + 0.1 * ratio * cos(starting_point(2)))* max_unit + min_x;
		ans[1].y = (starting_point(1) + 0.1 * ratio * sin(starting_point(2)))* max_unit + min_y;
		//std::cout << "ans[1] is " << ans[1].x << "," << ans[1].y << std::endl;
		return ans;
	}
	catch (char* ex) {
		std::cout << ex << std::endl;
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
	catch (...) {
		std::cout << "BFGS optimization failure." << std::endl;
	}

	return{};
}