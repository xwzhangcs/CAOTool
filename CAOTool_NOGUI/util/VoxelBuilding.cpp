#include "DisjointVoxelData.h"
#include "ContourUtils.h"

namespace util {

	int VoxelBuilding::voxelCountSum(int height) {
		int ans = 0;
		for (auto voxel_node : node_stack[height]) {
			ans += voxel_node->voxel_count;
		}
		return ans;
	}

	void VoxelBuilding::removeNode(int z, int index) {
		std::shared_ptr<VoxelNode> node = node_stack[z][index];

		for (auto it = node->children.begin(); it != node->children.end(); it++) {
			it->first->parents.erase(node);
			if (it->first->parents.size() == 0) {
				removeNode(z + 1, it->first);
			}
		}
		
		for (auto it = node->parents.begin(); it != node->parents.end(); it++) {
			it->first->children.erase(node);
		}

		node_stack[z].erase(node_stack[z].begin() + index);
	}

	void VoxelBuilding::removeNode(int z, std::shared_ptr<VoxelNode> node) {
		for (int i = 0; i < node_stack[z].size(); i++) {
			if (node_stack[z][i] == node) {
				removeNode(z, i);
				break;
			}
		}
	}

}