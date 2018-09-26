#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dlib/optimization.h>
#include "simp/BuildingSimplification.h"
#include "util/OBJWriter.h"
#include "util/TopFaceWriter.h"
#include "util/PlyWriter.h"
#include "util/ContourUtils.h"

class CrispAccurateOpt {

	typedef dlib::matrix<double, 0, 1> column_vector;

	class BFGSSolver {
	private:
		std::vector<util::VoxelBuilding> voxel_buildings;
		std::string jido_mash;
		std::vector<float> write_obj_info;
		std::map<int, std::vector<double>> algorithms;
		bool record_stats;
		int min_num_slices_per_layer;
		float alpha;
		float layering_threshold;
		float snapping_threshold;
		float orientation;
		float min_contour_area;
		float max_obb_ratio;
		bool allow_triangle_contour;
		bool allow_overhang; 
		float min_hole_ratio;
		std::vector<regularizer::Config> regularizer_configs;

	public:
		BFGSSolver(std::vector<util::VoxelBuilding>& voxel_buildings, std::string jido_mesh, std::vector<float> write_obj_info, std::map<int, std::vector<double>>& algorithms, bool record_stats, int min_num_slices_per_layer, float alpha, float layering_threshold, float snapping_threshold, float orientation, float min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, float min_hole_ratio, const std::vector<regularizer::Config>& regularizer_configs) {
			this->voxel_buildings = voxel_buildings;
			this->jido_mash = jido_mash;
			this->write_obj_info = write_obj_info;
			this->algorithms = algorithms;
			this->record_stats = record_stats;
			this->min_num_slices_per_layer = min_num_slices_per_layer;
			this->alpha = alpha;
			this->layering_threshold = layering_threshold;
			this->snapping_threshold = snapping_threshold;
			this->orientation = orientation;
			this->min_contour_area = min_contour_area;
			this->max_obb_ratio = max_obb_ratio;
			this->allow_triangle_contour = allow_triangle_contour;
			this->allow_overhang = allow_overhang;
			this->min_hole_ratio = min_hole_ratio;
			this->regularizer_configs = regularizer_configs;
		}

		double operator() (const column_vector& arg) const {
			// efficient ransac parameters
			std::vector<double> current_alg_parameters = algorithms.at(simp::BuildingSimplification::ALG_EFFICIENT_RANSAC);
			current_alg_parameters[8] = arg(0);
			current_alg_parameters[9] = arg(1);
			current_alg_parameters[10] = arg(2);
			current_alg_parameters[11] = arg(3);
			current_alg_parameters[12] = arg(4);

			current_alg_parameters[13] = arg(5);
			current_alg_parameters[14] = arg(6);
			std::map<int, std::vector<double>> current_algorithms;
			current_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC] = current_alg_parameters;
			{
				for (int i = 0; i < current_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC].size(); i++)
					std::cout << current_algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC][i] << std::endl;
			}
			// regularization parameters
			std::vector<regularizer::Config> current_regularizer_configs;
			current_regularizer_configs.resize(regularizer_configs.size());
			for (int i = 0; i < regularizer_configs.size(); i++)
				current_regularizer_configs[i] = regularizer_configs[i];

			current_regularizer_configs[0].angle_threshold_RA = arg(7);
			current_regularizer_configs[0].angle_threshold_parallel = arg(8);
			current_regularizer_configs[0].pointDisThreshold = arg(9);
			current_regularizer_configs[0].segDisThreshold = arg(10);
			current_regularizer_configs[0].segAngleThreshold = arg(11);
			/*{
				std::cout << "current_regularizer_configs[0].angle_threshold_RA " << current_regularizer_configs[0].angle_threshold_RA << std::endl;
				std::cout << "current_regularizer_configs[0].angle_threshold_parallel " << current_regularizer_configs[0].angle_threshold_parallel << std::endl;
				std::cout << "current_regularizer_configs[0].pointDisThreshold " << current_regularizer_configs[0].pointDisThreshold << std::endl;
				std::cout << "current_regularizer_configs[0].segDisThreshold " << current_regularizer_configs[0].segDisThreshold << std::endl;
				std::cout << "current_regularizer_configs[0].segAngleThreshold " << current_regularizer_configs[0].segAngleThreshold << std::endl;
			}*/
			//
			std::vector<util::VoxelBuilding> curren_voxel_buildings;
			curren_voxel_buildings.resize(voxel_buildings.size());
			for (int i = 0; i < voxel_buildings.size(); i++)
				curren_voxel_buildings[i] = voxel_buildings[i];
			try {
				float score = 0.0f;
				std::vector<std::shared_ptr<util::BuildingLayer>> buildings;
				buildings = simp::BuildingSimplification::simplifyBuildings(curren_voxel_buildings, current_algorithms, false, min_num_slices_per_layer, alpha, layering_threshold, snapping_threshold, orientation, min_contour_area, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio, current_regularizer_configs);
				// generate cgv obj file
				std::string cgv_mash = "../data/cgv.obj";
				util::obj::OBJWriter::write(cgv_mash, write_obj_info[0], write_obj_info[1], write_obj_info[2], write_obj_info[3], write_obj_info[4], write_obj_info[5], buildings);
				
				// generate layers from our models
				std::vector<std::string> cgv_layer_images = loadOBJ(cgv_mash, "../data/cgv_");
				// generate layers from jido models
				std::vector<std::string> jido_layer_images = loadOBJ(jido_mash, "../data/jido_");
				// find the matching layers
				std::vector<int> match_indices = FindMatachingLayers(jido_layer_images, cgv_layer_images);
				// compute similarity socre
				int layers_number = cgv_layer_images.size();
				int total_polygons = 0;
				for (int i = 0; i < layers_number; i++){
					std::vector<cv::Point2f> cgv_contour; 
					{
						cv::Mat src = cv::imread(cgv_layer_images[i], 0);
						std::vector<std::vector<cv::Point> > contours;
						std::vector<cv::Vec4i> hierarchy;
						/// Find contours
						findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
						float epsilon = 1.0f;
						// simplify contour
						cv::approxPolyDP(contours[0], cgv_contour, epsilon, true);
					}
					std::vector<cv::Point2f> jido_contour;
					{
						cv::Mat src = cv::imread(jido_layer_images[match_indices[i]], 0);
						std::vector<std::vector<cv::Point> > contours;
						std::vector<cv::Vec4i> hierarchy;
						/// Find contours
						findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
						float epsilon = 1.0f;
						// simplify contour
						cv::approxPolyDP(contours[0], jido_contour, epsilon, true);
					}
					// compute score
					score += util::calculateIOUbyCGAL(cgv_contour, jido_contour);
					total_polygons++;
				}
				score = score / total_polygons;
				return score;
			}
			catch (...) {
				std::cout << "exception" << std::endl;
				return 0;
			}
		}
	};

protected:
	CrispAccurateOpt();
	~CrispAccurateOpt();

public:
	static std::vector<std::shared_ptr<util::BuildingLayer>> fit(std::vector<util::VoxelBuilding>& voxel_buildings, std::string jido_mesh, std::vector<float> write_obj_info, std::map<int, std::vector<double>>& algorithms, bool record_stats, int min_num_slices_per_layer, float alpha, float layering_threshold, float snapping_threshold, float orientation, float min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, float min_hole_ratio, const std::vector<regularizer::Config>& regularizer_configs);
	
private:
	// Helper functions
	static int indexOfNumberLetter(std::string& str, int offset);
	static int lastIndexOfNumberLetter(std::string& str);
	static std::vector<std::string> split(const std::string &s, char delim);

	static int generateVectorForAllLayers(std::shared_ptr<util::BuildingLayer> root, int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship);
	static std::vector<std::string> loadOBJ(std::string filename, std::string save_path);
	static std::vector<int> FindMatachingLayers(std::vector<std::string> jido_layer_images, std::vector<std::string> cgv_layer_images);
};

