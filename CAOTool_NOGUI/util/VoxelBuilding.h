#pragma once

#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "BuildingLayer.h"
#include "ContourUtils.h"

namespace util {

	/**
	 * This class represents the connected voxels in the slice.
	 */
	class VoxelNode {
	public:
		int cluster_id;		// cluster id (mainly for debug)
		int height;
		int voxel_count;
		Polygon contour;
		std::unordered_map<std::shared_ptr<VoxelNode>, bool> parents;
		std::unordered_map<std::shared_ptr<VoxelNode>, bool> children;

	public:
		VoxelNode() {}
		VoxelNode(int cluster_id, int height) : cluster_id(cluster_id), height(height) {}
		VoxelNode(int cluster_id, int height, int voxel_count, const Polygon& contour) : cluster_id(cluster_id), height(height), voxel_count(voxel_count), contour(contour) {}
	};

	/**
	 * Voxel representation of the building using the graph structure.
	 */
	class VoxelBuilding {
	public:
		int building_id;
		std::vector<std::vector<std::shared_ptr<VoxelNode>>> node_stack;

	public:
		VoxelBuilding() : building_id(-1) {}
		VoxelBuilding(int building_id) : building_id(building_id) {}

		int voxelCountSum(int height);
		void removeNode(int z, int index);
		void removeNode(int z, std::shared_ptr<VoxelNode> node);
	};

}