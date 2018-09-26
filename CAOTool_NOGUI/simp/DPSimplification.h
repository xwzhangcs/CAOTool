#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "../util/BuildingLayer.h"
#include "../util/ContourUtils.h"

namespace simp {

	class DPSimplification {
	protected:
		DPSimplification() {}

	public:
		static util::Polygon simplify(const util::Polygon& polygon, float epsilon, float min_hole_ratio);
		static void decomposePolygon(util::Polygon& polygon);
	};

}