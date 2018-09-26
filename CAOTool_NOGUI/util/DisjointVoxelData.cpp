#include "DisjointVoxelData.h"
#include "ContourUtils.h"

namespace util {

	DisjointVoxelData::DisjointVoxelData() {
	}
	
	/**
	 * Disjoint the buildings in teh voxel data. 
	 * Each building of the resultant array represents the graph structure of the connected components in the slices.
	 * A few heuristics are applied to remove noise and generate cleaner buildings.
	 * - Too small connected components will be discarded.
	 * - Too small contours will be discarded.
	 */
	std::vector<VoxelBuilding> DisjointVoxelData::disjoint(const std::vector<cv::Mat_<uchar>>& voxel_data, int voxel_value_threshold, float min_voxel_count_ratio) {
		// initialize the clustering
		std::vector<cv::Mat_<short>> building_clustering(voxel_data.size());
		for (int h = 0; h < voxel_data.size(); h++) {
			building_clustering[h] = cv::Mat_<short>(voxel_data[h].size(), -1);
		}

		// Cluster the connected components in the voxel data.
		// The clustering information will be stored in building_clustering.
		setbuf(stdout, NULL);
		printf("Clustering the voxel data...\n");
		int building_id = 0;
		int max_voxel_count = 0;
		std::vector<int> voxel_counts;	// this array stores the number of voxels for each cluster.
		for (int h = 0; h < voxel_data.size(); h++) {
			for (int r = 0; r < voxel_data[h].rows; r++) {
				for (int c = 0; c < voxel_data[h].cols; c++) {
					if (voxel_data[h](r, c) > voxel_value_threshold && building_clustering[h](r, c) == -1) {
						int voxel_count = disjointBuilding(voxel_data, building_clustering, r, c, h, building_id, voxel_value_threshold);
						voxel_counts.push_back(voxel_count);
						max_voxel_count = std::max(max_voxel_count, voxel_count);
						building_id++;
					}
				}
			}
		}

		int num_buildings = building_id;

		// Construct a graph structure for each connected component.
		// The node of the graph represents a connected component in each slice.
		std::vector<VoxelBuilding> buildings;
		for (int i = 0; i < num_buildings; i++) {
			if (voxel_counts[i] < max_voxel_count * min_voxel_count_ratio) continue;
			VoxelBuilding building_voxels = constructGraph(building_clustering, i);
			buildings.push_back(building_voxels);
		}

		return buildings;
	}

	/**
	 * Layer the building based on the threshold.
	 * Similar contour of slices will be merged into one layer.
	 * 
	 */
	std::vector<std::shared_ptr<BuildingLayer>> DisjointVoxelData::layering(const util::VoxelBuilding& building_voxels, float threshold, int min_num_slices_per_layer) {
		std::vector<std::shared_ptr<BuildingLayer>> bottom_building_layers;

		// layer buildings
		for (int z = 0; z < building_voxels.node_stack.size(); z++) {
			for (int i = 0; i < building_voxels.node_stack[z].size(); i++) {
				if (building_voxels.node_stack[z][i]->parents.size() == 0) {
					std::vector<std::shared_ptr<BuildingLayer>> layers = layeringBuilding(building_voxels.building_id, building_voxels.node_stack[z][i], threshold, min_num_slices_per_layer);
					bottom_building_layers.insert(bottom_building_layers.end(), layers.begin(), layers.end());
				}
			}
		}

		return bottom_building_layers;
	}

	/**
	 * Construct a graph structure of the building.
	 * Each connected component in the slice will be a node of the graph. Too small contours will be discarded at this moment.
	 */
	VoxelBuilding DisjointVoxelData::constructGraph(const std::vector<cv::Mat_<short>>& building_clustering, int building_id) {
		VoxelBuilding building_voxels(building_id);

		std::vector<std::shared_ptr<VoxelNode>> all_nodes;

		// initialize the clustering
		std::vector<cv::Mat_<int>> clustering(building_clustering.size());
		for (int h = 0; h < building_clustering.size(); h++) {
			clustering[h] = cv::Mat_<int>(building_clustering[h].size(), -1);
		}

		// perform 3D dfs to cluster the voxel data
		int cluster_id = 0;
		int max_voxel_count = 0;
		std::vector<int> voxel_counts;	// this array stores the number of voxels for each cluster.
		for (int h = 0; h < building_clustering.size(); h++) {
			std::vector<std::shared_ptr<VoxelNode>> voxel_nodes_in_slice;
			int max_voxel_count = 0;

			// find connected components
			for (int r = 0; r < building_clustering[h].rows; r++) {
				for (int c = 0; c < building_clustering[h].cols; c++) {
					if (building_clustering[h](r, c) == building_id && clustering[h](r, c) == -1) {
						// create a voxel node
						std::shared_ptr<VoxelNode> voxel_node(new VoxelNode(cluster_id, h));
						all_nodes.push_back(voxel_node);
						voxel_nodes_in_slice.push_back(voxel_node);

						voxel_node->voxel_count = clusterBuilding(building_clustering[h], building_id, clustering, r, c, h, cluster_id, all_nodes);
						int max_voxel_count = std::max(max_voxel_count, voxel_node->voxel_count);

						// extract contour
						voxel_node->contour = getPolygonFromCluster(clustering[h], cluster_id);
						convertCoordinatesOfPolygon(voxel_node->contour, clustering[0].cols, clustering[0].rows);

						cluster_id++;
					}
				}
			}

			building_voxels.node_stack.push_back(voxel_nodes_in_slice);
		}

		// Find the height where the voxel count sum is the largest
		int max_voxel_count_sum = 0;
		int max_voxel_count_height = -1;
		for (int i = 0; i < building_voxels.node_stack.size(); i++) {
			int voxel_count_sum = building_voxels.voxelCountSum(i);
			if (voxel_count_sum > max_voxel_count_sum) {
				max_voxel_count_sum = voxel_count_sum;
				max_voxel_count_height = i;
			}
		}

		// Start scanning downward to find the noisy slices at the bottom
		for (int i = max_voxel_count_height - 1; i >= 0; i--) {
			int voxel_count_sum = building_voxels.voxelCountSum(i);
			if (voxel_count_sum < max_voxel_count_sum * 0.5) {
				// remove the noisy small bottom slices
				building_voxels.node_stack.erase(building_voxels.node_stack.begin(), building_voxels.node_stack.begin() + i + 1);

				// clear the parent edges
				for (auto voxel_node : building_voxels.node_stack[0]) {
					voxel_node->parents.clear();
				}

				break;
			}
		}

		// remove relatively small contours in each slice
		for (int z = 0; z < building_voxels.node_stack.size(); z++) {
			if (building_voxels.node_stack[z].size() == 0) continue;

			int max_area = 0;
			for (int i = 0; i < building_voxels.node_stack[z].size(); i++) {
				max_area = std::max(max_area, building_voxels.node_stack[z][i]->voxel_count);
			}

			for (int i = building_voxels.node_stack[z].size() - 1; i >= 0; i--) {
				if (building_voxels.node_stack[z][i]->voxel_count < max_area * 0.1) {
					building_voxels.removeNode(z, i);
				}
			}
		}

		// Convert the graph to tree for now by removing extra parent nodes.
		// This should work for now, but the graph structure may be desired in the future.
		for (int z = 1; z < building_voxels.node_stack.size(); z++) {
			for (int i = 0; i < building_voxels.node_stack[z].size(); i++) {
				if (building_voxels.node_stack[z][i]->parents.size() > 1) {
					int max_area = 0;
					std::shared_ptr<VoxelNode> max_parent_node;
					for (auto it = building_voxels.node_stack[z][i]->parents.begin(); it != building_voxels.node_stack[z][i]->parents.end(); it++) {
						if (it->first->voxel_count > max_area) {
							max_area = it->first->voxel_count;
							max_parent_node = it->first;
						}
					}

					building_voxels.node_stack[z][i]->parents.clear();
					building_voxels.node_stack[z][i]->parents[max_parent_node] = true;
				}
			}
		}

		return building_voxels;
	}

	/**
	 * Traverse the connected voxels with value greater than the threshold and are not visited yet.
	 *
	 * @param voxel_data			voxel data
	 * @param building_clustering	save the clustering information
	 * @param r						row of current voxel
	 * @param c						column of current voxel
	 * @param h						height of current voxel
	 * @param building_id			building id
	 * @param voxel_value_threshold	threshold
	 * @return						the voxel count of the connected component
	 */
	int DisjointVoxelData::disjointBuilding(const std::vector<cv::Mat_<uchar>>&voxel_data, std::vector<cv::Mat_<short>>& building_clustering, int r, int c, int h, int building_id, int voxel_value_threshold) {
		const std::vector<std::pair<int, std::pair<int, int>>> dirs = { { 0, { 1, 0 } }, { 0, { -1, 0 } }, { 0, { 0, 1 } }, { 0, { 0, -1 } }, { 1, { 0, 0 } }, { -1, { 0, 0 } } };

		int R = voxel_data[0].rows;
		int C = voxel_data[0].cols;

		std::queue<std::pair<int, std::pair<int, int>>> Q;
		Q.push({ h, { r, c } });

		building_clustering[h](r, c) = building_id;
		int count = 0;

		while (!Q.empty()) {
			int h = Q.front().first;
			int r = Q.front().second.first;
			int c = Q.front().second.second;
			Q.pop();
			count++;

			for (int i = 0; i < dirs.size(); i++) {
				int h2 = h + dirs[i].first;
				int r2 = r + dirs[i].second.first;
				int c2 = c + dirs[i].second.second;

				if (r2 >= 0 && r2 < R && c2 >= 0 && c2 < C && h2 >= 0 && h2 < voxel_data.size() && voxel_data[h2](r2, c2) > voxel_value_threshold && building_clustering[h2](r2, c2) == -1) {
					building_clustering[h2](r2, c2) = building_id;
					Q.push({ h2, { r2, c2 } });
				}
			}
		}

		return count;
	}

	/**
	 * Traverse the voxels with specified building id in the slice and are not visited yet.
	 *
	 * @param building_clustering	building clustering data (only the voxels with value of building_id will be traversed)
	 * @param building_id			building id
	 * @param clustering			save the clustering information
	 * @param r						row of current voxel
	 * @param c						column of current voxel
	 * @param h						height of current voxel
	 * @param cluster_id			current cluster id
	 * @param voxel_nodes			generated voxel nodes
	 * @return						the voxel count of the connected component
	 */
	int DisjointVoxelData::clusterBuilding(const cv::Mat_<short>& building_clustering, int building_id, std::vector<cv::Mat_<int>>&clustering, int r, int c, int h, int cluster_id, std::vector<std::shared_ptr<VoxelNode>>& voxel_nodes) {
		const std::vector<std::pair<int, int>> dirs = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };

		int R = building_clustering.rows;
		int C = building_clustering.cols;

		std::queue<std::pair<int, int>> Q;
		Q.push({ r, c });

		clustering[h](r, c) = cluster_id;
		int count = 0;

		while (!Q.empty()) {
			int r = Q.front().first;
			int c = Q.front().second;
			Q.pop();
			count++;
			if (h > 0 && clustering[h - 1](r, c) >= 0) {
				// update the edges
				int parent_cluster_id = clustering[h - 1](r, c);
				voxel_nodes[parent_cluster_id]->children[voxel_nodes[cluster_id]] = true;
				voxel_nodes[cluster_id]->parents[voxel_nodes[parent_cluster_id]] = true;
			}

			for (int i = 0; i < dirs.size(); i++) {
				int r2 = r + dirs[i].first;
				int c2 = c + dirs[i].second;

				if (r2 >= 0 && r2 < R && c2 >= 0 && c2 < C && building_clustering(r2, c2) == building_id && clustering[h](r2, c2) == -1) {
					clustering[h](r2, c2) = cluster_id;
					Q.push({ r2, c2 });
				}
			}
		}

		return count;
	}

	cv::Mat_<uchar> DisjointVoxelData::getSliceOfCluster(const cv::Mat_<int>& clustering, int cluster_id, int& min_x, int& min_y, int& max_x, int& max_y) {
		min_x = std::numeric_limits<int>::max();
		min_y = std::numeric_limits<int>::max();
		max_x = 0;
		max_y = 0;

		bool found = false;

		cv::Mat_<uchar> ans = cv::Mat_<uchar>::zeros(clustering.size());
		for (int r = 0; r < clustering.rows; r++) {
			for (int c = 0; c < clustering.cols; c++) {
				if (clustering(r, c) == cluster_id) {
					ans(r, c) = 255;
					min_x = std::min(min_x, c);
					min_y = std::min(min_y, r);
					max_x = std::max(max_x, c);
					max_y = std::max(max_y, r);
					found = true;
				}
			}
		}

		if (!found) throw "No specified cluster was found.";

		return ans;
	}

	Polygon DisjointVoxelData::getPolygonFromCluster(const cv::Mat_<int>& clustering, int cluster_id) {
		int min_x, min_y, max_x, max_y;
		cv::Mat_<uchar> slice = getSliceOfCluster(clustering, cluster_id, min_x, min_y, max_x, max_y);
		cv::Mat_<uchar> roi_slice(slice, cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1));
		std::vector<Polygon> polygons = findContours(roi_slice, true);
		for (int i = 0; i < polygons.size(); i++) {
			polygons[i].translate(min_x, min_y);
		}

		return polygons[0];
	}

	/**
	 * Convert the coordinates of polygon such that the center of the slice will be the origin
	 * and the y direction is upward.
	 *
	 * @param polygon	polygon
	 * @param width		width of the slice
	 * @param height	height of the slice
	 */
	void DisjointVoxelData::convertCoordinatesOfPolygon(Polygon& polygon, int width, int height) {
		// convert the polygon coordinates to the world coordinate system
		for (int i = 0; i < polygon.contour.size(); i++) {
			polygon.contour[i].x = polygon.contour[i].x - width / 2;
			polygon.contour[i].y = height / 2 - polygon.contour[i].y;
		}
		for (auto& hole : polygon.holes) {
			for (int j = 0; j < hole.size(); j++) {
				hole[j].x = hole[j].x - width / 2;
				hole[j].y = height / 2 - hole[j].y;
			}
		}
	}
	
	/**
	 * Layer the building.
	 */
	std::vector<std::shared_ptr<BuildingLayer>> DisjointVoxelData::layeringBuilding(int building_id, const std::shared_ptr<VoxelNode>& voxel_node, float threshold, int min_num_slices_per_layer) {
		// create the bottom layer
		std::shared_ptr<BuildingLayer> bottom_building_layer(std::shared_ptr<BuildingLayer>(new BuildingLayer(building_id, voxel_node->height, voxel_node->height + 1)));
		bottom_building_layer->raw_footprints.push_back({ voxel_node->contour });

		// initialize queue
		std::queue<std::pair<std::shared_ptr<VoxelNode>, std::shared_ptr<BuildingLayer>>> Q;
		Q.push({ voxel_node, bottom_building_layer });

		while (!Q.empty()) {
			std::shared_ptr<VoxelNode> voxel_node = Q.front().first;
			std::shared_ptr<BuildingLayer> layer = Q.front().second;
			Q.pop();

			std::vector<Polygon> cur_polygon;
			cur_polygon.push_back(voxel_node->contour);

			std::unordered_map<std::shared_ptr<VoxelNode>, bool> child_nodes = voxel_node->children;

			while (child_nodes.size() > 0) {
				std::vector<Polygon> child_polygons;
				for (auto it = child_nodes.begin(); it != child_nodes.end(); it++) {
					child_polygons.push_back(it->first->contour);
				}

				if (calculateIOU(cur_polygon, child_polygons) >= threshold) {
					// merge nodes
					std::unordered_map<std::shared_ptr<VoxelNode>, bool> old_children = child_nodes;
					child_nodes.clear();
					layer->raw_footprints.push_back({});
					for (auto it = old_children.begin(); it != old_children.end(); it++) {
						for (auto it2 = it->first->children.begin(); it2 != it->first->children.end(); it2++) {
							child_nodes[it2->first] = true;
						}
						layer->raw_footprints.back().push_back(it->first->contour);
						layer->top_height = it->first->height + 1;
					}
				}
				else {
					// create new layers
					for (auto it = child_nodes.begin(); it != child_nodes.end(); it++) {
						std::shared_ptr<BuildingLayer> child_layer(std::shared_ptr<BuildingLayer>(new BuildingLayer(building_id, it->first->height, it->first->height + 1)));
						child_layer->raw_footprints.push_back({ it->first->contour });
						layer->children.push_back(child_layer);
						Q.push({ it->first, child_layer });
					}

					break;
				}
			}
		}

		std::shared_ptr<BuildingLayer> root_layer(std::shared_ptr<BuildingLayer>(new BuildingLayer(building_id, -1, 0)));
		root_layer->children.push_back(bottom_building_layer);

		// Merge too thin layer to the one beneath
		removeThinLayers(root_layer, min_num_slices_per_layer);

		return root_layer->children;
	}

	/**
	* Remove too thin layer by merging it to the one beneath.
	*
	* @param layer		the bottom layer of the building
	*/
	void DisjointVoxelData::removeThinLayers(std::shared_ptr<BuildingLayer> layer, int min_num_slices_per_layer) {
		for (int i = layer->children.size() - 1; i >= 0; i--) {
			removeThinLayers(layer->children[i], min_num_slices_per_layer);

			if (layer->children[i]->top_height - layer->children[i]->bottom_height < min_num_slices_per_layer) {
				if (layer->children[i]->children.size() == 0) {
					// remove too thin layer if it does not have any children.
					layer->children.erase(layer->children.begin() + i);
				}
				else {
					// merge the child with its children
					std::shared_ptr<BuildingLayer> child_layer = layer->children[i];
					layer->children.erase(layer->children.begin() + i);
					for (auto grandchild_layer : child_layer->children) {
						layer->children.insert(layer->children.begin() + i, grandchild_layer);
						grandchild_layer->bottom_height = child_layer->bottom_height;
						//grandchild_layer->raw_footprints.insert(grandchild_layer->raw_footprints.begin(), child_layer->raw_footprints.begin(), child_layer->raw_footprints.end());

						// add the bottom slice of the grandchild layer for N times, where N is the number of the slices in the child layer
						for (int j = 0; j < child_layer->raw_footprints.size(); j++) {
							grandchild_layer->raw_footprints.insert(grandchild_layer->raw_footprints.begin(), grandchild_layer->raw_footprints[0]);
						}
					}
				}
			}
		}
	}

}