#include "ShapeFitLayersAll.h"
#include "SymmetryLineDetector.h"

ShapeFitLayersAll::ShapeFitLayersAll() {
}

ShapeFitLayersAll::~ShapeFitLayersAll() {
}

void ShapeFitLayersAll::fit(std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship, regularizer::Config configuration)
{
	bool bUseIntra = configuration.bUseIntra;
	float intraWeight = configuration.intraWeight;
	bool bUseInter = configuration.bUseInter;
	float interWeight = configuration.interWeight;
	bool bUseRaOpt = configuration.bUseRaOpt;
	float angle_threshold_RA = configuration.angle_threshold_RA;
	float raWeight = configuration.raWeight;
	bool bUseParallelOpt = configuration.bUseParallelOpt;
	float angle_threshold_parallel = configuration.angle_threshold_parallel;
	float parallelWeight = configuration.parallelWeight;
	bool bUseSymmetryLineOpt = configuration.bUseSymmetryLineOpt;
	float symmetryIouThreshold = configuration.symmetryIouThreshold;
	std::vector<layer_polygons> layers_symmetry_lines;
	float symmetryWeight = configuration.symmetryWeight;
	bool bUseAccuracyOpt = configuration.bUseAccuracyOpt;
	float accuracyWeight = configuration.accuracyWeight;
	bool bUsePointSnapOpt = configuration.bUsePointSnapOpt;
	float pointDisThreshold = configuration.pointDisThreshold;
	float pointWeight = configuration.pointWeight;
	bool bUseSegSnapOpt = configuration.bUseSegSnapOpt;
	float segDisThreshold = configuration.segDisThreshold;
	float segAngleThreshold = configuration.segAngleThreshold;
	float segWeight = configuration.segWeight;
	if (!bUseInter && !bUseIntra){
		return;
	}
	/*{
		std::cout << "bUseRa " << bUseRaOpt << " ra angle is " << angle_threshold_RA << " ra weight is " << raWeight << std::endl;
		std::cout << "bUseParallel " << bUseParallelOpt << " Parallel angle is " << angle_threshold_parallel << " Parallel weight is " << parallelWeight << std::endl;
		std::cout << "bUseSymmetry " << bUseSymmetryLineOpt << " Symmetry weight is " << symmetryWeight << std::endl;
		std::cout << "bUseAccuracy " << bUseAccuracyOpt << " Accuracy weight is " << accuracyWeight << std::endl;
	}
	{
		std::cout << "bUsePoint " << bUsePointSnapOpt << " Point threshold is " << pointDisThreshold << " Point weight is " << pointWeight << std::endl;
		std::cout << "bUseSeg " << bUseSegSnapOpt << " seg angle is " << segAngleThreshold << " seg weight is " << segWeight << std::endl;
	}*/
	//check weights
	if (bUseInter || bUseIntra){
		float weight = 0.0f;
		if (bUseInter)
			weight += interWeight;
		if (bUseIntra)
			weight += intraWeight;
		if (abs(weight - 1.0f) < 0.0001)
		{
			//
		}
		else{
			std::cout << "Please check intra and inter weight assignment!!!" << std::endl;
			return;
		}
	}
	if (bUseIntra){
		//check the sum weight equals 1
		float weight = 0.0f;
		if (bUseRaOpt)
			weight += raWeight;
		if (bUseParallelOpt)
			weight += parallelWeight;
		if (bUseSymmetryLineOpt)
			weight += symmetryWeight;
		if (bUseAccuracyOpt)
			weight += accuracyWeight;
		if (abs(weight - 1.0f) < 0.0001)
		{
			//
		}
		else{
			std::cout << "Please check intra weight assignment!!!" << std::endl;
			return;
		}
	}
	if (bUseInter){
		float weight = 0.0f;
		if (bUsePointSnapOpt)
			weight += pointWeight;
		if (bUseSegSnapOpt)
			weight += segWeight;
		if (abs(weight - 1.0f) < 0.0001)
		{
			//
		}
		else{
			std::cout << "Please check inter weight assignment!!!" << std::endl;
			return;
		}
	}

	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	for (int k = 0; k < layers.size(); k++){
		for (int i = 0; i < layers[k]->presentativeContours.size(); i++) {
			for (int j = 0; j < layers[k]->presentativeContours[i].contour.size(); j++){
				min_x = std::min(min_x, layers[k]->presentativeContours[i].contour[j].x);
				min_y = std::min(min_y, layers[k]->presentativeContours[i].contour[j].y);
				max_x = std::max(max_x, layers[k]->presentativeContours[i].contour[j].x);
				max_y = std::max(max_y, layers[k]->presentativeContours[i].contour[j].y);
			}
		}
	}
	for (int k = 0; k < layers.size(); k++){
		for (int i = 0; i < layers[k]->footprints.size(); i++) {
			for (int j = 0; j < layers[k]->footprints[i].contour.size(); j++){
				min_x = std::min(min_x, layers[k]->footprints[i].contour[j].x);
				min_y = std::min(min_y, layers[k]->footprints[i].contour[j].y);
				max_x = std::max(max_x, layers[k]->footprints[i].contour[j].x);
				max_y = std::max(max_y, layers[k]->footprints[i].contour[j].y);
			}
		}
	}
	if (bUseSymmetryLineOpt){
		layers_symmetry_lines.resize(layers.size());
		for (int k = 0; k < layers.size(); k++){
			layers_symmetry_lines[k].resize(layers[k]->presentativeContours.size());
			for (int i = 0; i < layers[k]->presentativeContours.size(); i++){
				std::vector<cv::Point2f> computed_symmetry_line = SymmetryLineDetector::fitSymmetryLine(layers[k]->presentativeContours[i].contour.points);
				std::vector<cv::Point2f> symmetry_polygon;
				for (int j = 0; j < layers[k]->presentativeContours[i].contour.size(); j++) {
					symmetry_polygon.push_back(util::mirrorPoint(computed_symmetry_line[0], computed_symmetry_line[1], layers[k]->presentativeContours[i].contour[j]));
				}

				// calculate IOU between mirror polygon and original polygon
				float similarity_iou = util::calculateIOU(layers[k]->presentativeContours[i].contour.points, symmetry_polygon);
				if (similarity_iou >= symmetryIouThreshold * 0.01){
					layers_symmetry_lines[k][i] = computed_symmetry_line;
				}
				else{
					layers_symmetry_lines[k][i].clear();
				}
			}
		}

		for (int k = 0; k < layers.size(); k++){
			for (int i = 0; i < layers_symmetry_lines[k].size(); i++){
				for (int j = 0; j < layers_symmetry_lines[k][i].size(); j++){
					min_x = std::min(min_x, layers_symmetry_lines[k][i][j].x);
					min_y = std::min(min_y, layers_symmetry_lines[k][i][j].y);
					max_x = std::max(max_x, layers_symmetry_lines[k][i][j].x);
					max_y = std::max(max_y, layers_symmetry_lines[k][i][j].y);
				}
			}
		}
	}
	float max_unit = std::max(max_x - min_x + 1.0f, max_y - min_y + 1.0f);
	regularizer::Config config(bUseIntra, intraWeight, bUseInter, interWeight, bUseRaOpt, angle_threshold_RA, raWeight, bUseParallelOpt, angle_threshold_parallel, parallelWeight, bUseSymmetryLineOpt, symmetryIouThreshold, symmetryWeight, bUseAccuracyOpt, accuracyWeight, bUsePointSnapOpt, pointDisThreshold / max_unit, pointWeight, bUseSegSnapOpt, segDisThreshold / max_unit, segAngleThreshold, segWeight);

	std::vector<layer_polygons> normalized_polygons(layers.size());
	for (int k = 0; k < layers.size(); k++){
		normalized_polygons[k].resize(layers[k]->presentativeContours.size());
		for (int i = 0; i < layers[k]->presentativeContours.size(); i++) {
			normalized_polygons[k][i].resize(layers[k]->presentativeContours[i].contour.size());
			for (int j = 0; j < layers[k]->presentativeContours[i].contour.size(); j++){
				normalized_polygons[k][i][j] = cv::Point2f((layers[k]->presentativeContours[i].contour[j].x - min_x) / max_unit, (layers[k]->presentativeContours[i].contour[j].y - min_y) / max_unit);
			}
		}
	}

	// normalized_symmetry_line
	std::vector<layer_polygons> normalized_symmetry_lines;
	if (bUseSymmetryLineOpt){
		normalized_symmetry_lines.resize(layers.size());
		for (int k = 0; k < layers.size(); k++){
			normalized_symmetry_lines[k].resize(layers_symmetry_lines[k].size());
			for (int i = 0; i < layers_symmetry_lines[k].size(); i++) {
				normalized_symmetry_lines[k][i].resize(layers_symmetry_lines[k][i].size());
				for (int j = 0; j < layers_symmetry_lines[k][i].size(); j++){
					normalized_symmetry_lines[k][i][j] = cv::Point2f((layers_symmetry_lines[k][i][j].x - min_x) / max_unit, (layers_symmetry_lines[k][i][j].y - min_y) / max_unit);
				}
			}
		}
	}
	int total_points = 0;
	std::vector<layer_polygons> normalized_polygons_init(layers.size());
	for (int k = 0; k < layers.size(); k++){
		normalized_polygons_init[k].resize(layers[k]->footprints.size());
		for (int i = 0; i < layers[k]->footprints.size(); i++) {
			normalized_polygons_init[k][i].resize(layers[k]->footprints[i].contour.size());
			total_points += layers[k]->footprints[i].contour.size();
			for (int j = 0; j < layers[k]->footprints[i].contour.size(); j++){
				normalized_polygons_init[k][i][j] = cv::Point2f((layers[k]->footprints[i].contour[j].x - min_x) / max_unit, (layers[k]->footprints[i].contour[j].y - min_y) / max_unit);
			}
		}
	}
	std::vector<std::pair<float, float>> layers_height;
	layers_height.resize(layers.size());
	for (int k = 0; k < layers.size(); k++){
		layers_height[k] = std::make_pair(layers[k]->top_height, layers[k]->bottom_height);
	}
	// tree info
	std::vector<std::pair<std::vector<int>, std::vector<int>>> tree_info;
	tree_info.resize(layers.size());
	for (int k = 0; k < layers.size(); k++){
		for (int i = 0; i < layers_relationship.size(); i++){
			if (layers_relationship[i].first == k){
				tree_info[k].second.push_back(layers_relationship[i].second);
			}
			if (layers_relationship[i].second == k){
				tree_info[k].first.push_back(layers_relationship[i].first);
			}
		}
	}

	// initial
	std::vector<std::vector<std::vector<bool>>> validity_layer_polygons;
	validity_layer_polygons.resize(layers.size());
	for (int k = 0; k < layers.size(); k++){
		validity_layer_polygons[k].resize(layers[k]->footprints.size());
		for (int i = 0; i < layers[k]->footprints.size(); i++){
			validity_layer_polygons[k][i].push_back(false);// RA
			validity_layer_polygons[k][i].push_back(false);// Parallel
			validity_layer_polygons[k][i].push_back(false);// Symmetry
			validity_layer_polygons[k][i].push_back(false);// Accuracy
			validity_layer_polygons[k][i].push_back(false);// Point
			validity_layer_polygons[k][i].push_back(false);// Seg
		}
	}
	// check validity of RA
	if (bUseRaOpt){
		bool bValid_per_layer = false;
		bool bValid = false;
		for (int k = 0; k < layers.size(); k++){
			bValid_per_layer = false;
			for (int i = 0; i < layers[k]->footprints.size(); i++){
				if (layers[k]->footprints[i].contour.size() != 0){
					if (validRA(layers[k]->footprints[i].contour.points, bUseRaOpt, angle_threshold_RA)){
						bValid_per_layer = true;
						validity_layer_polygons[k][i][0] = true;
					}
				}
			}
			if (bValid_per_layer){
				bValid = true;
			}
		}
		if (!bValid){
			config.bUseRaOpt = false;
			std::cout << "RA invalid" << std::endl;
		}
	}
	// check validity of Parallel
	if (bUseParallelOpt){
		bool bValid_per_layer = false;
		bool bValid = false;
		for (int k = 0; k < layers.size(); k++){
			bValid_per_layer = false;
			for (int i = 0; i < layers[k]->footprints.size(); i++){
				if (layers[k]->footprints[i].contour.size() != 0){
					if (validParallel(layers[k]->footprints[i].contour.points, bUseParallelOpt, angle_threshold_parallel)){
						bValid_per_layer = true;
						validity_layer_polygons[k][i][1] = true;
					}
				}
			}
			if (bValid_per_layer){
				bValid = true;
			}
		}
		if (!bValid){
			config.bUseParallelOpt = false;
			std::cout << "Parallel invalid" << std::endl;
		}
	}
	// check validity of symmetry
	if (bUseSymmetryLineOpt){
		bool bValid_per_layer = false;
		bool bValid = false;
		for (int k = 0; k < layers_symmetry_lines.size(); k++){
			bValid_per_layer = false;
			for (int i = 0; i < layers_symmetry_lines[k].size(); i++) {
				if (layers_symmetry_lines[k][i].size() > 0){
					bValid_per_layer = true;
					validity_layer_polygons[k][i][2] = true;
				}
			}
			if (bValid_per_layer){
				bValid = true;
			}
		}
		if (!bValid){
			config.bUseSymmetryLineOpt = false;
			std::cout << "Symmetry invalid" << std::endl;
		}
	}
	// check validity of accuracy
	if (bUseAccuracyOpt){
		for (int k = 0; k < layers.size(); k++){
			for (int i = 0; i < layers[k]->footprints.size(); i++) {
				validity_layer_polygons[k][i][3] = true;
			}
		}
	}

	// check validity of point
	if (bUsePointSnapOpt){
		bool bValid_per_layer = false;
		bool bValid = false;
		for (int k = 0; k < layers.size(); k++){
			bValid_per_layer = false;
			for (int i = 0; i < layers[k]->footprints.size(); i++){
				if (tree_info[k].first.size() == 0)
				{
					bValid_per_layer = false;
					break;
				}
				for (int j = 0; j < tree_info[k].first.size(); j++){
					if (validPointOpt(layers[k]->footprints[i], layers[tree_info[k].first[j]]->footprints, pointDisThreshold)){
						bValid_per_layer = true;
						validity_layer_polygons[k][i][4] = true;
						break;
					}
				}
			}
			if (bValid_per_layer){
				bValid = true;
			}
		}
		if (!bValid){
			config.bUsePointSnapOpt = false;
			std::cout << "PointSnap invalid" << std::endl;
		}
	}

	// check validity of seg
	if (bUseSegSnapOpt){
		bool bValid_per_layer = false;
		bool bValid = false;
		for (int k = 0; k < layers.size(); k++){
			bValid_per_layer = false;
			for (int i = 0; i < layers[k]->footprints.size(); i++){
				if (tree_info[k].first.size() == 0)
				{
					bValid_per_layer = false;
					break;
				}
				for (int j = 0; j < tree_info[k].first.size(); j++){
					if (validSegOpt(layers[k]->footprints[i], layers[tree_info[k].first[j]]->footprints, segDisThreshold, segAngleThreshold)){
						bValid_per_layer = true;
						validity_layer_polygons[k][i][5] = true;
						break;
					}
				}
			}
			if (bValid_per_layer){
				bValid = true;
			}
		}
		if (!bValid){
			config.bUseSegSnapOpt = false;
			std::cout << "SegSnap invalid" << std::endl;
		}
	}
	// debug
	//for (int k = 0; k < layers.size(); k++){
	//	for (int i = 0; i < layers[k]->footprints.size(); i++){
	//		std::cout << "layer " << k << " polygon " << i << ": ";
	//		for (int j = 0; j < 6; j++)
	//			std::cout << validity_layer_polygons[k][i][j] << ", ";
	//		std::cout << std::endl;
	//	}
	//}
	// 
	if (!config.bUseRaOpt && !config.bUseParallelOpt && !config.bUseSymmetryLineOpt && !config.bUseAccuracyOpt && !config.bUsePointSnapOpt && !config.bUseSegSnapOpt)
	{
		std::cout << "no need to do optimization!!!" << std::endl;
		return;
	}
	try {
		//std::cout << "total points is " << total_points << std::endl;
		column_vector starting_point(total_points * 2);
		int start_index = 0;
		for (int k = 0; k < normalized_polygons_init.size(); k++){
			for (int i = 0; i < normalized_polygons_init[k].size(); i++){
				for (int j = 0; j < normalized_polygons_init[k][i].size(); j++){
					starting_point((j + start_index) * 2) = normalized_polygons_init[k][i][j].x;
					starting_point((j + start_index) * 2 + 1) = normalized_polygons_init[k][i][j].y;
				}
				start_index += normalized_polygons_init[k][i].size();
			}
		}
		BFGSSolver solver(normalized_polygons, normalized_polygons_init, normalized_symmetry_lines, layers_height, tree_info, validity_layer_polygons, config);
		if (true)
			find_max_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), solver, starting_point, 1, 0.0001);
		else
			find_min_using_approximate_derivatives(dlib::lbfgs_search_strategy(total_points), dlib::objective_delta_stop_strategy(1e-6), solver, starting_point, 0, 0.00001);
		start_index = 0;
		std::vector<layer_polygons> new_layers_contours;
		new_layers_contours.resize(layers.size());
		for (int k = 0; k < layers.size(); k++){
			new_layers_contours[k].resize(layers[k]->footprints.size());
			for (int i = 0; i < layers[k]->footprints.size(); i++) {
				new_layers_contours[k][i].resize(layers[k]->footprints[i].contour.size());
				for (int j = 0; j < layers[k]->footprints[i].contour.size(); j++){
					new_layers_contours[k][i][j].x = starting_point((j + start_index) * 2) * max_unit + min_x;
					new_layers_contours[k][i][j].y = starting_point((j + start_index) * 2 + 1) * max_unit + min_y;
				}
				start_index += layers[k]->footprints[i].contour.size();
			}
		}
		// reset contours
		for (int k = 0; k < layers.size(); k++){
			for (int i = 0; i < layers[k]->footprints.size(); i++)
				layers[k]->footprints[i].contour = new_layers_contours[k][i];
		}
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

}

bool ShapeFitLayersAll::validRA(const std::vector<cv::Point2f>& polygon, bool bUseRaOpt, int ra_angle_threshold){
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
			if (abs(angle_init - 45) <= ra_angle_threshold || abs(angle_init - 90) <= ra_angle_threshold || abs(angle_init - 135) <= ra_angle_threshold){
				return true;
			}
		}
		return false;
	}
	return false;
}

bool ShapeFitLayersAll::validParallel(const std::vector<cv::Point2f>& polygon, bool bUseParallelOpt, int parallel_angle_threshold){
	int total_segments = polygon.size();
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
			//	if (abs(angle_init) <= parallel_angle_threshold || abs(angle_init - 180) <= parallel_angle_threshold)
			//		return true;
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
			if (abs(angle_init) <= parallel_angle_threshold || abs(angle_init - 180) <= parallel_angle_threshold){
				return true;
			}
		}
		return false;
	}
	return false;
}

bool ShapeFitLayersAll::validPointOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, float dis_threshold){
	for (int k = 0; k < src_polygon.size(); k++){// current source polygon
		for (int i = 0; i < des_layer_polygons.size(); i++){// all polygons in the destination layer
			for (int j = 0; j < des_layer_polygons[i].size(); j++){// one polygon in the destination layer
				cv::Point2f src_p = src_polygon[k];
				cv::Point2f des_p = des_layer_polygons[i][j];
				float dis = cv::norm(src_p - des_p);
				if (dis <= dis_threshold)
					return true;
				//std::cout << "dis_init from " << k << " to " << j << " is " << dis << std::endl;
			}
		}
	}
	return false;
}

bool ShapeFitLayersAll::validSegOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, float dis_threshold, float angle_threshold){
	int total_seg_src = src_polygon.size();
	for (int k = 0; k < total_seg_src; k++){// current source polygon
		for (int i = 0; i < des_layer_polygons.size(); i++){// all polygons in the destination layer
			int total_seg_des = des_layer_polygons[i].size();
			for (int j = 0; j < total_seg_des; j++){// one polygon in the destination layer
				// angle score
				cv::Point2f src_start = src_polygon[k];
				cv::Point2f src_end = src_polygon[(k + 1) % total_seg_src];
				cv::Point2f des_start = des_layer_polygons[i][j];
				cv::Point2f des_end = des_layer_polygons[i][(j + 1) % total_seg_des];
				float angle = util::lineLineAngle(src_start, src_end, des_start, des_end);
				float dis = util::distance(src_start, src_end, des_start, des_end);
				angle = std::min(abs(angle), abs(180 - angle));
				if (angle <= angle_threshold && dis <= dis_threshold)
					return true;
			}
		}
	}
	return false;
}

bool ShapeFitLayersAll::validPointOpt(const util::Polygon& src_polygon, const std::vector<util::Polygon>& des_layer_polygons, float dis_threshold){
	for (int k = 0; k < src_polygon.contour.size(); k++){// current source polygon
		for (int i = 0; i < des_layer_polygons.size(); i++){// all polygons in the destination layer
			for (int j = 0; j < des_layer_polygons[i].contour.size(); j++){// one polygon in the destination layer
				cv::Point2f src_p = src_polygon.contour[k];
				cv::Point2f des_p = des_layer_polygons[i].contour[j];
				float dis = cv::norm(src_p - des_p);
				if (dis <= dis_threshold)
					return true;
				//std::cout << "dis_init from " << k << " to " << j << " is " << dis << std::endl;
			}
		}
	}
	return false;
}

bool ShapeFitLayersAll::validSegOpt(const util::Polygon& src_polygon, const std::vector<util::Polygon>& des_layer_polygons, float dis_threshold, float angle_threshold){
	int total_seg_src = src_polygon.contour.size();
	for (int k = 0; k < total_seg_src; k++){// current source polygon
		for (int i = 0; i < des_layer_polygons.size(); i++){// all polygons in the destination layer
			int total_seg_des = des_layer_polygons[i].contour.size();

			for (int j = 0; j < total_seg_des; j++){// one polygon in the destination layer
				// angle score
				cv::Point2f src_start = src_polygon.contour[k];
				cv::Point2f src_end = src_polygon.contour[(k + 1) % total_seg_src];
				cv::Point2f des_start = des_layer_polygons[i].contour[j];
				cv::Point2f des_end = des_layer_polygons[i].contour[(j + 1) % total_seg_des];
				float angle = util::lineLineAngle(src_start, src_end, des_start, des_end);
				float dis = util::distance(src_start, src_end, des_start, des_end);
				angle = std::min(abs(angle), abs(180 - angle));
				//std::cout << "angle is " << angle <<", dis is "<< dis << std::endl;
				if (angle <= angle_threshold && dis <= dis_threshold)
					return true;
			}
		}
	}
	return false;
}