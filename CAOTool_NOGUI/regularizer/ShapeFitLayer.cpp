#include "ShapeFitLayer.h"
#include "SymmetryLineDetector.h"


ShapeFitLayer::ShapeFitLayer() {
}

ShapeFitLayer::~ShapeFitLayer() {
}

std::vector<util::Polygon> ShapeFitLayer::fit(const std::vector<util::Polygon>& polygons, const std::vector<util::Polygon>& ini_points, regularizer::Config config)
{
	bool bUseIntra = config.bUseIntra;
	float intraWeight = config.intraWeight;
	bool bUseRaOpt = config.bUseRaOpt;
	float angle_threshold_RA = config.angle_threshold_RA;
	float raWeight = config.raWeight;
	bool bUseParallelOpt = config.bUseParallelOpt;
	float angle_threshold_parallel = config.angle_threshold_parallel;
	float parallelWeight = config.parallelWeight;
	bool bUseSymmetryLineOpt = config.bUseSymmetryLineOpt;
	float symmetryIouThreshold = config.symmetryIouThreshold;
	std::vector<std::vector<cv::Point2f>> symmetry_lines;
	float symmetryWeight = config.symmetryWeight;
	bool bUseAccuracyOpt = config.bUseAccuracyOpt;
	float accuracyWeight = config.accuracyWeight;
	/*{
		std::cout << "bUseRa " << bUseRaOpt << " ra angle is " << angle_threshold_RA << " ra weight is " << raWeight << std::endl;
		std::cout << "bUseParallel " << bUseParallelOpt << " Parallel angle is " << angle_threshold_parallel << " Parallel weight is " << parallelWeight << std::endl;
		std::cout << "bUseSymmetry " << bUseSymmetryLineOpt << " Symmetry threshold is " << symmetryIouThreshold << " Symmetry weight is " << symmetryWeight << std::endl;
		std::cout << "bUseAccuracy " << bUseAccuracyOpt << " Accuracy weight is " << accuracyWeight << std::endl;
	}*/
	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();

	for (int i = 0; i < polygons.size(); i++) {
		for (int j = 0; j < polygons[i].contour.size(); j++){
			min_x = std::min(min_x, polygons[i].contour[j].x);
			min_y = std::min(min_y, polygons[i].contour[j].y);
			max_x = std::max(max_x, polygons[i].contour[j].x);
			max_y = std::max(max_y, polygons[i].contour[j].y);
		}
	}
	for (int i = 0; i < ini_points.size(); i++) {
		for (int j = 0; j < ini_points[i].contour.size(); j++){
			min_x = std::min(min_x, ini_points[i].contour[j].x);
			min_y = std::min(min_y, ini_points[i].contour[j].y);
			max_x = std::max(max_x, ini_points[i].contour[j].x);
			max_y = std::max(max_y, ini_points[i].contour[j].y);
		}
	}
	if (bUseSymmetryLineOpt){
		// find symmetry lines
		symmetry_lines.resize(polygons.size());
		for (int i = 0; i < polygons.size(); i++){
			std::vector<cv::Point2f> computed_symmetry_line = SymmetryLineDetector::fitSymmetryLine(polygons[i].contour.points);
			std::vector<cv::Point2f> symmetry_polygon;
			for (int j = 0; j < polygons[i].contour.size(); j++) {
				symmetry_polygon.push_back(util::mirrorPoint(computed_symmetry_line[0], computed_symmetry_line[1], polygons[i].contour[j]));
			}

			// calculate IOU between mirror polygon and original polygon
			float similarity_iou = util::calculateIOU(polygons[i].contour.points, symmetry_polygon);
			if (similarity_iou >= symmetryIouThreshold * 0.01){
				//std::cout << "Find symmetry line" << std::endl;
				symmetry_lines[i] = computed_symmetry_line;
			}
			else{
				//std::cout << "Can't find symmetry line" << std::endl;
				symmetry_lines[i].clear();
			}
		}
		
		for (int i = 0; i < symmetry_lines.size(); i++){
			for (int j = 0; j < symmetry_lines[i].size(); j++){
				min_x = std::min(min_x, symmetry_lines[i][j].x);
				min_y = std::min(min_y, symmetry_lines[i][j].y);
				max_x = std::max(max_x, symmetry_lines[i][j].x);
				max_y = std::max(max_y, symmetry_lines[i][j].y);
			}
		}
	}
	float max_unit = std::max(max_x - min_x, max_y - min_y);
	std::vector<std::vector<cv::Point2f>> normalized_polygons(polygons.size());
	for (int i = 0; i < polygons.size(); i++) {
		normalized_polygons[i].resize(polygons[i].contour.size());
		for (int j = 0; j < polygons[i].contour.size(); j++){
			normalized_polygons[i][j] = cv::Point2f((polygons[i].contour[j].x - min_x) / max_unit, (polygons[i].contour[j].y - min_y) / max_unit);
		}
	}

	// normalized_symmetry_line
	std::vector<std::vector<cv::Point2f>> normalized_symmetry_lines;
	int valid_symmetry_lines = 0;
	if (bUseSymmetryLineOpt){
		normalized_symmetry_lines.resize(symmetry_lines.size());
		for (int i = 0; i < symmetry_lines.size(); i++) {
			normalized_symmetry_lines[i].resize(symmetry_lines[i].size());
			valid_symmetry_lines += symmetry_lines[i].size();
			for (int j = 0; j < symmetry_lines[i].size(); j++){
				normalized_symmetry_lines[i][j] = cv::Point2f((symmetry_lines[i][j].x - min_x) / max_unit, (symmetry_lines[i][j].y - min_y) / max_unit);
			}
		}
	}

	int total_points = 0;
	std::vector<std::vector<cv::Point2f>> normalized_polygons_init(ini_points.size());
	for (int i = 0; i < ini_points.size(); i++) {
		normalized_polygons_init[i].resize(ini_points[i].contour.size());
		total_points += ini_points[i].contour.size();
		for (int j = 0; j < ini_points[i].contour.size(); j++){
			normalized_polygons_init[i][j] = cv::Point2f((ini_points[i].contour[j].x - min_x) / max_unit, (ini_points[i].contour[j].y - min_y) / max_unit);
		}
	}

	// we need to check whether there are valid angles
	if (bUseRaOpt || bUseParallelOpt){
		bool bValid = false;
		for (int i = 0; i < ini_points.size(); i++){
			if (ini_points[i].contour.size() != 0){
				if (validRAorParallel(ini_points[i].contour.points, bUseRaOpt, angle_threshold_RA, bUseParallelOpt, angle_threshold_parallel)){
					bValid = true;
					break;
				}
			}
		}
		if (!bValid && !bUseSymmetryLineOpt && !bUseAccuracyOpt){
			std::cout << "No need to do optimizaiton for RA and parallel" << std::endl;
			return ini_points;
		}
	}
	if (bUseSymmetryLineOpt && !bUseParallelOpt && !bUseAccuracyOpt && !bUseRaOpt && valid_symmetry_lines == 0){
		std::cout << "No need to do optimizaiton for symmetry" << std::endl;
		return ini_points;
	}

	try {
		//std::cout << "total points is " << total_points << std::endl;
		column_vector starting_point(total_points * 2);
		int start_index = 0;
		for (int i = 0; i < normalized_polygons_init.size(); i++){
			for (int j = 0; j < normalized_polygons_init[i].size(); j++){
				starting_point((j + start_index) * 2) = normalized_polygons_init[i][j].x;
				starting_point((j + start_index) * 2 + 1) = normalized_polygons_init[i][j].y;
			}
			start_index += normalized_polygons_init[i].size();
		}

		BFGSSolver solver(normalized_polygons, normalized_polygons_init, bUseRaOpt, angle_threshold_RA, raWeight, bUseParallelOpt, angle_threshold_parallel, parallelWeight, bUseSymmetryLineOpt, normalized_symmetry_lines, symmetryWeight, bUseAccuracyOpt, accuracyWeight);
		find_max_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-4), solver, starting_point, 1, 0.0001);
		std::vector<std::vector<cv::Point2f>> ans(ini_points.size());
		std::vector<util::Polygon> ans_polygons(ini_points.size());
		start_index = 0;
		for (int i = 0; i < ini_points.size(); i++) {
			ans[i].resize(ini_points[i].contour.size());
			ans_polygons[i] = ini_points[i];
			for (int j = 0; j < ini_points[i].contour.size(); j++){
				ans[i][j].x = starting_point((j + start_index) * 2) * max_unit + min_x;
				ans[i][j].y = starting_point((j + start_index) * 2 + 1) * max_unit + min_y;
			}
			ans_polygons[i].contour = ans[i];
			start_index += ini_points[i].contour.size();
		}
		return ans_polygons;
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

bool ShapeFitLayer::validRAorParallel(const std::vector<cv::Point2f>& polygon, bool bUseRaOpt, int ra_angle_threshold, bool bUseParallelOpt, int parallel_angle_threshold){
	int total_segments = polygon.size();
	if (bUseRaOpt){
		for (int i = 0; i < total_segments; i++){
			int first_start = i;
			int first_end = (i + 1) % total_segments;
			int second_start = (i + 1) % total_segments;
			int second_end = (i + 2) % total_segments;

			// init angles
			cv::Point2f a_init = cv::Point2f(polygon[first_start].x, polygon[first_start].y);
			cv::Point2f b_init = cv::Point2f(polygon[first_end].x, polygon[first_end].y);
			cv::Point2f c_init = cv::Point2f(polygon[second_start].x, polygon[second_start].y);;
			cv::Point2f d_init = cv::Point2f(polygon[second_end].x, polygon[second_end].y);
			float angle_init = util::lineLineAngle(a_init, b_init, c_init, d_init);
			//std::cout << i << " angle is " << angle_init << std::endl;
			// check
			if (abs(angle_init - 45) >= 0.001 && abs(angle_init - 45) <= ra_angle_threshold){
				return true;
			}
			if (abs(angle_init - 90) >= 0.001 && abs(angle_init - 90) <= ra_angle_threshold){
				return true;
			}
			if (abs(angle_init - 135) >= 0.001 && abs(angle_init - 135) <= ra_angle_threshold){
				return true;
			}
		}
		return false;
	}
	if (bUseParallelOpt){
		for (int i = 0; i < total_segments; i++){
			//for (int j = i + 1; j < total_segments; j++){
			//	int first_start = i;
			//	int first_end = (i + 1) % total_segments;
			//	int second_start = j;
			//	int second_end = (j + 1) % total_segments;

			//	// init angles
			//	cv::Point2f a_init = cv::Point2f(polygon[first_start].x, polygon[first_start].y);
			//	cv::Point2f b_init = cv::Point2f(polygon[first_end].x, polygon[first_end].y);
			//	cv::Point2f c_init = cv::Point2f(polygon[second_start].x, polygon[second_start].y);;
			//	cv::Point2f d_init = cv::Point2f(polygon[second_end].x, polygon[second_end].y);
			//	float angle_init = util::lineLineAngle(a_init, b_init, c_init, d_init);
			//	//std::cout << i <<" to "<<j <<" angle is " << angle_init << std::endl;
			//	// check
			//	if (abs(angle_init) <= parallel_angle_threshold && abs(angle_init) >= 0.001){
			//		return true;
			//	}
			//	if (abs(angle_init - 180) <= parallel_angle_threshold && abs(angle_init - 180) >= 0.001){
			//		return true;
			//	}
			//}
			int first_start = i;
			int first_end = (i + 1) % total_segments;
			int second_start = (i + 1) % total_segments;
			int second_end = (i + 2) % total_segments;

			// init angles
			cv::Point2f a_init = cv::Point2f(polygon[first_start].x, polygon[first_start].y);
			cv::Point2f b_init = cv::Point2f(polygon[first_end].x, polygon[first_end].y);
			cv::Point2f c_init = cv::Point2f(polygon[second_start].x, polygon[second_start].y);;
			cv::Point2f d_init = cv::Point2f(polygon[second_end].x, polygon[second_end].y);
			float angle_init = util::lineLineAngle(a_init, b_init, c_init, d_init);
			//std::cout << i << " angle is " << angle_init << std::endl;
			// check
			if (abs(angle_init) <= parallel_angle_threshold && abs(angle_init) >= 0.001){
				return true;
			}
			if (abs(angle_init - 180) <= parallel_angle_threshold && abs(angle_init - 180) >= 0.001){
				return true;
			}
		}
		return false;
	}
	return false;
}