#pragma once

#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "BuildingLayer.h"
#include "ContourUtils.h"
#include "VoxelBuilding.h"

namespace util {

	class DisjointVoxelData {
	protected:
		DisjointVoxelData();

	public:
		static std::vector<VoxelBuilding> disjoint(const std::vector<cv::Mat_<uchar>>& voxel_data, int voxel_value_threshold = 128, float min_voxel_count_ratio = 0.1);
		static std::vector<std::shared_ptr<BuildingLayer>> layering(const util::VoxelBuilding& building_voxels, float threshold, int min_num_slices_per_layer);

	private:
		static VoxelBuilding constructGraph(const std::vector<cv::Mat_<short>>& building_clustering, int building_id);
		static int disjointBuilding(const std::vector<cv::Mat_<uchar>>& voxel_data, std::vector<cv::Mat_<short>>& building_clustering, int r, int c, int h, int building_id, int voxel_value_threshold);
		static int clusterBuilding(const cv::Mat_<short>& building_clustering, int building_id, std::vector<cv::Mat_<int>>& clustering, int r, int c, int h, int cluster_id, std::vector<std::shared_ptr<VoxelNode>>& voxel_nodes);
		static cv::Mat_<uchar> getSliceOfCluster(const cv::Mat_<int>& clustering, int cluster_id, int& min_x, int& min_y, int& max_x, int& max_y);
		static Polygon getPolygonFromCluster(const cv::Mat_<int>& clustering, int cluster_id);
		static void convertCoordinatesOfPolygon(Polygon& polygon, int width, int height);
		static std::vector<std::shared_ptr<BuildingLayer>> layeringBuilding(int building_id, const std::shared_ptr<VoxelNode>& voxel_node, float threshold, int min_num_slices_per_layer);
		static void removeThinLayers(std::shared_ptr<BuildingLayer> layer, int min_num_slices_per_layer);
	};

}