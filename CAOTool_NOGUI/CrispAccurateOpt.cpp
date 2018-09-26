#include "CrispAccurateOpt.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

CrispAccurateOpt::CrispAccurateOpt() {
}

CrispAccurateOpt::~CrispAccurateOpt() {
}

std::vector<std::shared_ptr<util::BuildingLayer>> CrispAccurateOpt::fit(std::vector<util::VoxelBuilding>& voxel_buildings, std::string jido_mesh, std::vector<float> write_obj_info, std::map<int, std::vector<double>>& algorithms, bool record_stats, int min_num_slices_per_layer, float alpha, float layering_threshold, float snapping_threshold, float orientation, float min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, float min_hole_ratio, const std::vector<regularizer::Config>& regularizer_configs)
{
	int total_points = 0;
	// read parameters of detection
	int curve_num_iterations = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[0];
	int curve_min_points = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[1];
	double curve_max_error_ratio_to_radius = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[2];
	double curve_cluster_epsilon = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[3];
	double curve_min_angle = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[4];
	double curve_min_radius = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[5];
	double curve_max_radius = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[6];

	int line_num_iterations = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[7];
	double line_min_points = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[8];
	double line_max_error = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[9];
	double line_cluster_epsilon = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[10];
	double line_min_length = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[11];
	double line_angle_threshold = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[12];

	double contour_max_error = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[13];
	double contour_angle_threshold = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC)[14];

	total_points += 7;
	// read parameters of regularization
	// RA
	double ra_weight = regularizer_configs[0].raWeight;
	double ra_angle = regularizer_configs[0].angle_threshold_RA;
	// Parallel
	double parallel_weight = regularizer_configs[0].parallelWeight;
	double parallel_angle = regularizer_configs[0].angle_threshold_parallel;
	
	// Symmetry 
	//double symmetry_weight;
	//double symmetry_ratio;
	// Accuracy
	//double accuracy_weight;
	// Point Snap
	double pointSnap_weight = regularizer_configs[0].pointWeight;
	double pointSnap_dis = regularizer_configs[0].pointDisThreshold;
	// Seg Snap
	double segSnap_weight = regularizer_configs[0].segWeight;
	double segSnap_dis = regularizer_configs[0].segDisThreshold;
	double segSnap_angle = regularizer_configs[0].segAngleThreshold;
	/*{
		std::cout << "ra_weight is " << ra_weight << ", ra_angle is " << ra_angle << std::endl;
		std::cout << "parallel_weight is " << parallel_weight << ", parallel_angle is " << parallel_angle << std::endl;
		std::cout << "pointSnap_weight is " << pointSnap_weight << ", pointSnap_dis is " << pointSnap_dis << std::endl;
		std::cout << "segSnap_weight is " << segSnap_weight << ", segSnap_dis is " << segSnap_dis << std::endl;
	}*/
	total_points += 5;
	try {
		column_vector starting_point(total_points);

		// parameters from line detection
		starting_point(0) = line_min_points;
		starting_point(1) = line_max_error;
		starting_point(2) = line_cluster_epsilon;
		starting_point(3) = line_min_length;
		starting_point(4) = line_angle_threshold;

		// parameters from contour generation
		starting_point(5) = contour_max_error;
		starting_point(6) = contour_angle_threshold;

		// parameters from regularization
		starting_point(7) = ra_angle;
		starting_point(8) = parallel_angle;
		starting_point(9) = pointSnap_dis;
		starting_point(10) = segSnap_dis;
		starting_point(11) = segSnap_angle;

		BFGSSolver solver(voxel_buildings, jido_mesh, write_obj_info, algorithms, record_stats, min_num_slices_per_layer, alpha, layering_threshold, snapping_threshold, orientation, min_contour_area, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio, regularizer_configs);
		find_max_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-6), solver, starting_point, 1, 0.0001);
		std::vector<std::shared_ptr<util::BuildingLayer>> buildings;
		std::map<int, std::vector<double>> ans_algorithms;
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC] = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][8] = starting_point(0);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][9] = starting_point(1);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][10] = starting_point(2);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][11] = starting_point(3);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][12] = starting_point(4);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][13] = starting_point(5);
		ans_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][14] = starting_point(6);

		std::vector<regularizer::Config> ans_regularizer_configs;
		ans_regularizer_configs.resize(regularizer_configs.size());
		for (int i = 0; i < regularizer_configs.size(); i++)
			ans_regularizer_configs[i] = regularizer_configs[i];
		ans_regularizer_configs[0].angle_threshold_RA = starting_point(7);
		ans_regularizer_configs[0].angle_threshold_parallel = starting_point(8);
		ans_regularizer_configs[0].pointDisThreshold = starting_point(9);
		ans_regularizer_configs[0].segDisThreshold = starting_point(10);
		ans_regularizer_configs[0].segAngleThreshold = starting_point(11);
		/*{
			std::cout << "ans_regularizer_configs[0].angle_threshold_RA " << ans_regularizer_configs[0].angle_threshold_RA << std::endl;
			std::cout << "ans_regularizer_configs[0].angle_threshold_parallel " << ans_regularizer_configs[0].angle_threshold_parallel << std::endl;
			std::cout << "ans_regularizer_configs[0].pointDisThreshold " << ans_regularizer_configs[0].pointDisThreshold << std::endl;
			std::cout << "ans_regularizer_configs[0].segDisThreshold " << ans_regularizer_configs[0].segDisThreshold << std::endl;
			std::cout << "ans_regularizer_configs[0].segAngleThreshold " << ans_regularizer_configs[0].segAngleThreshold << std::endl;
		}*/
		buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, ans_algorithms, false, min_num_slices_per_layer, alpha, layering_threshold, snapping_threshold, orientation, min_contour_area, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio, ans_regularizer_configs);

		return buildings;
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

/**
* Generate images for all layers/
*
* @param		root
* @param		image index
*/
int CrispAccurateOpt::generateVectorForAllLayers(std::shared_ptr<util::BuildingLayer> root, int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship){
	std::shared_ptr<util::BuildingLayer> building = std::shared_ptr<util::BuildingLayer>(new util::BuildingLayer(root->building_id, root->footprints, root->bottom_height, root->top_height));
	building->presentativeContours = root->presentativeContours;
	layers.push_back(building);
	int current_layer_id = layer_id;
	for (auto child_layer : root->children){
		layer_id++;
		layers_relationship.push_back(std::make_pair(current_layer_id, layer_id));
		layer_id = generateVectorForAllLayers(child_layer, layer_id, layers, layers_relationship);
	}
	return layer_id;
}

int CrispAccurateOpt::indexOfNumberLetter(std::string& str, int offset) {
	for (int i = offset; i < str.length(); ++i) {
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '-' || str[i] == '.') return i;
	}
	return str.length();
}

int CrispAccurateOpt::lastIndexOfNumberLetter(std::string& str) {
	for (int i = str.length() - 1; i >= 0; --i) {
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '-' || str[i] == '.') return i;
	}
	return 0;
}

std::vector<std::string> CrispAccurateOpt::split(const std::string &s, char delim) {
	std::vector<std::string> elems;

	std::stringstream ss(s);
	std::string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}

	return elems;
}

std::vector<std::string> CrispAccurateOpt::loadOBJ(std::string filename, std::string save_path) {
	if (!filename.empty()) {
		std::ifstream file(filename);
		if (!file.is_open()) {
			std::vector<std::string> empty;
			return empty;
		}

		// Store vertex
		std::vector<glm::vec3> raw_vertices;
		std::vector<std::pair<float, glm::vec2>> layer_vertices;
		std::vector<glm::vec3> face_vertex_indices;
		std::vector<float> layer_heights;

		std::string line;
		while (getline(file, line)) {
			if (line.substr(0, 2) == "v ") {
				// Read position data
				int index1 = indexOfNumberLetter(line, 2);
				int index2 = lastIndexOfNumberLetter(line);
				std::vector<std::string> values = split(line.substr(index1, index2 - index1 + 1), ' ');
				glm::vec3 vert(stof(values[0]), stof(values[1]), stof(values[2]));
				glm::vec2 vert2d(stof(values[0]), stof(values[1]));
				float z_value = stof(values[2]);
				//std::cout << "x is " << vert.x << ", y is " << vert.y << ", z is " << vert.z << std::endl;
				raw_vertices.push_back(vert);
				layer_vertices.push_back(std::make_pair(z_value, vert2d));
			}
			else if (line.substr(0, 3) == "vn ") {
				// Read normal data

			}
			else if (line.substr(0, 2) == "f ") {
				// Read face data
				// Read face data
				int index1 = indexOfNumberLetter(line, 2);
				int index2 = lastIndexOfNumberLetter(line);
				std::vector<std::string> values = split(line.substr(index1, index2 - index1 + 1), ' ');
				for (int i = 0; i < values.size() - 2; i++) {
					// Split up vertex indices
					std::vector<std::string> v1 = split(values[0], '/');		// Triangle fan for ngons
					std::vector<std::string> v2 = split(values[i + 1], '/');
					std::vector<std::string> v3 = split(values[i + 2], '/');

					// Store position indices
					glm::vec3 vert(stoi(v1[0]) - 1, stoi(v2[0]) - 1, stoi(v3[0]) - 1);
					face_vertex_indices.push_back(vert);
					//std::cout << "1st index is " << vert.x << ", 2nd is " << vert.y << ", 3rd is " << vert.z << std::endl;
				}
			}
		}
		file.close();

		//std::cout << "layer_vertices size is " << layer_vertices.size() << std::endl;
		sort(layer_vertices.begin(), layer_vertices.end(), [](const std::pair<float, glm::vec2>& lhs, std::pair<float, glm::vec2>& rhs){return lhs.first < rhs.first; });
		for (int i = 0; i < layer_vertices.size(); i++)
		{
			if (layer_heights.size() == 0)
				layer_heights.push_back(layer_vertices[i].first);
			else{
				if (layer_vertices[i].first - layer_heights[layer_heights.size() - 1] >= 0.6)
					layer_heights.push_back(layer_vertices[i].first);
			}
		}
		// output layer height
		for (int i = 0; i < layer_heights.size(); i++){
			std::cout << "layer " << i << " height is " << layer_heights[i] << std::endl;
		}
		// earse the ground layer
		layer_heights.erase(layer_heights.begin());
		//
		std::vector<std::vector<glm::vec3>> layer_faces;
		layer_faces.resize(layer_heights.size());
		for (int i = 0; i < face_vertex_indices.size(); i++){
			int first = face_vertex_indices[i].x;
			int second = face_vertex_indices[i].y;
			int third = face_vertex_indices[i].z;
			//if (raw_vertices[first].z == raw_vertices[second].z && raw_vertices[first].z == raw_vertices[third].z){
			if (abs(raw_vertices[first].z - raw_vertices[second].z) < 0.6 && abs(raw_vertices[first].z - raw_vertices[third].z) < 0.6){
				for (int j = 0; j < layer_heights.size(); j++){
					//if (raw_vertices[first].z - layer_heights[j] < 0.6)
					if (raw_vertices[first].z >= layer_heights[j])
						layer_faces[j].push_back(face_vertex_indices[i]);
				}
			}
		}
		// get min_x min_y max_x max_y
		float min_x = std::numeric_limits<float>::max();
		float min_y = std::numeric_limits<float>::max();
		float max_x = -std::numeric_limits<float>::max();
		float max_y = -std::numeric_limits<float>::max();

		for (int i = 0; i < raw_vertices.size(); i++) {
			min_x = std::min(min_x, raw_vertices[i].x);
			min_y = std::min(min_y, raw_vertices[i].y);
			max_x = std::max(max_x, raw_vertices[i].x);
			max_y = std::max(max_y, raw_vertices[i].y);
		}
		min_x = min_x - 5;
		min_y = min_y - 5;
		max_x = max_x + 5;
		max_y = max_y + 5;
		int rows = max_y - min_y;
		int cols = max_x - min_x;
		int lineType = 8;
		std::vector<std::string> layer_image_files;
		for (int i = 0; i < layer_faces.size(); i++){
			cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC3);
			//std::cout << "layer faces " << i << ", size is " << layer_faces[i].size() << std::endl;
			for (int j = 0; j < layer_faces[i].size(); j++){
				cv::Point rook_points[1][3] = { 0 };
				const cv::Point* ppt[1];
				rook_points[0][0] = cv::Point(raw_vertices[layer_faces[i][j].x].x - min_x, raw_vertices[layer_faces[i][j].x].y - min_y);
				rook_points[0][1] = cv::Point(raw_vertices[layer_faces[i][j].y].x - min_x, raw_vertices[layer_faces[i][j].y].y - min_y);
				rook_points[0][2] = cv::Point(raw_vertices[layer_faces[i][j].z].x - min_x, raw_vertices[layer_faces[i][j].z].y - min_y);
				int npt[1] = { 3 };
				ppt[0] = rook_points[0];
				cv::fillPoly(img,
					ppt,
					npt,
					1,
					cv::Scalar(255, 255, 255),
					lineType);
			}

			std::string img_filename = save_path + std::to_string(i) + ".png";
			cv::imwrite(img_filename, img);
			layer_image_files.push_back(img_filename);
		}
		return layer_image_files;
	}
}

std::vector<int> CrispAccurateOpt::FindMatachingLayers(std::vector<std::string> files_jido, std::vector<std::string> files_cgv){
	// get layer images for jido model
	std::vector<cv::Mat_<uchar>> layer_imgs_jido(files_jido.size());
	for (int i = 0; i < files_jido.size(); i++) {
		layer_imgs_jido[i] = cv::imread(files_jido[i], cv::IMREAD_GRAYSCALE);
	}
	// get layer images for cgv model
	std::vector<cv::Mat_<uchar>> layer_imgs_cgv(files_cgv.size());
	for (int i = 0; i < files_cgv.size(); i++) {
		layer_imgs_cgv[i] = cv::imread(files_cgv[i], cv::IMREAD_GRAYSCALE);
	}

	// get new image size 
	int rows_jido = layer_imgs_jido[0].rows;
	int cols_jido = layer_imgs_jido[0].cols;
	// resize cgv images
	for (int i = 0; i < layer_imgs_cgv.size(); i++){
		cv::Mat img_tmp;
		cv::resize(layer_imgs_cgv[i], img_tmp, cv::Size(cols_jido, rows_jido), 0, 0, cv::INTER_NEAREST);
		layer_imgs_cgv[i] = img_tmp;
		//QString img_filename = "../data/cgv_" + QString::number(i) + ".png";
		//cv::imwrite(img_filename.toUtf8().constData(), layer_imgs_cgv[i]);
	}

	// find intersection between jido and cgv models
	std::vector<int> match_layer_indices;
	for (int i = 0; i < layer_imgs_cgv.size(); i++){
		cv::Mat_<uchar> img2 = layer_imgs_cgv[i];
		float best_score = -std::numeric_limits<float>::max();
		int best_index = -1;
		for (int j = 0; j < layer_imgs_jido.size(); j++){
			// computer IOU
			float score = 0.0f;
			int inter_cnt = 0;
			int union_cnt = 0;
			cv::Mat_<uchar> img1 = layer_imgs_jido[j];
			for (int r = 0; r < img1.rows; r++) {
				for (int c = 0; c < img1.cols; c++) {
					if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
					if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
				}
			}
			if (union_cnt == 0){
				score = 0.0f;
			}
			else
				score = (double)inter_cnt / union_cnt;
			if (score > best_score){
				best_score = score;
				best_index = j;
			}

		}
		//std::cout << "best_score is " << best_score << std::endl;
		//std::cout << "best_index is " << best_index << std::endl;
		match_layer_indices.push_back(best_index);
		// find the maximum

	}
	return match_layer_indices;
}
