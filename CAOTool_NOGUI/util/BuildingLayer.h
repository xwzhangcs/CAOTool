#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "ContourUtils.h"

namespace util {

	class BuildingLayer {
	public:
		int building_id;
		std::vector<std::vector<util::Polygon>> raw_footprints;
		std::vector<util::Polygon> footprints;
		std::vector<util::Polygon> presentativeContours;
		float bottom_height;
		float top_height;
		std::vector<std::shared_ptr<BuildingLayer>> children;

	public:
		BuildingLayer() {}
		BuildingLayer(int building_id, float bottom_height, float top_height);
		BuildingLayer(int building_id, const std::vector<util::Polygon>& footprints, float bottom_height, float top_height);

		std::vector<util::Polygon> selectRepresentativeContours();
		int getTopHeight();
	};

}