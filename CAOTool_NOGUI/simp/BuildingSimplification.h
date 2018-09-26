#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "../util/BuildingLayer.h"
#include "../util/DisjointVoxelData.h"
#include "../regularizer/Config.h"

namespace simp {
	
	class BuildingSimplification {
	public:
		enum ALGORITHM { ALG_UNKNOWN = 0, ALG_ALL, ALG_DP, ALG_RIGHTANGLE, ALG_CURVE, ALG_CURVE_RIGHTANGLE, ALG_EFFICIENT_RANSAC };

	protected:
		BuildingSimplification() {}

	public:
		static std::vector<std::shared_ptr<util::BuildingLayer>> simplifyBuildings(std::vector<util::VoxelBuilding>& voxel_buildings, std::map<int, std::vector<double>>& algorithms, bool record_stats, int min_num_slices_per_layer, float alpha, float layering_threshold, float snapping_threshold, float orientation, float min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, float min_hole_ratio, const std::vector<regularizer::Config>& regularizer_configs = std::vector<regularizer::Config>());
		static std::vector<std::shared_ptr<util::BuildingLayer>> simplifyBuildings(std::vector<util::VoxelBuilding>& voxel_buildings, int algorithm, bool record_stats, int min_num_slices_per_layer, float alpha, float layering_threshold, float epsilon, int resolution, float curve_threshold, float angle_threshold, float min_hole_ratio);
		
	private:
		static std::shared_ptr<util::BuildingLayer> simplifyBuildingByAll(int building_id, std::shared_ptr<util::BuildingLayer> layer, const std::vector<util::Polygon>& parent_contours, std::map<int, std::vector<double>>& algorithms, float alpha, float snapping_threshold, float orientation, float min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, float min_hole_ratio, bool curve_preferred, std::vector<std::tuple<float, long long, int>>& records);
		static std::vector<float> calculateCost(const util::Polygon& simplified_polygon, const util::Polygon& polygon, int height);
		static int generateVectorForAllLayers(std::shared_ptr<util::BuildingLayer> root, int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship);
		static void generateBuildingFromAllLayers(std::shared_ptr<util::BuildingLayer> root, int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship);
		static void generateImagesForAllLayers(std::shared_ptr<util::BuildingLayer> building, int index, std::string prefix);
		static std::shared_ptr<util::BuildingLayer> regularizerBuilding(std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship, const std::vector<regularizer::Config>& regularizer_configs, float snapping_threshold);
		static void post_processing(std::shared_ptr<util::BuildingLayer>& layer, float angle_threshold, float dis_threshold, bool bCurve);
		static void postSegSnapping(int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship, float snapping_threshold);
		static void postPointSnapping(int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship, float snapping_threshold);
		static void postOverhang(int layer_id, std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship);
	};

}
