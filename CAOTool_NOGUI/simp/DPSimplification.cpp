#include "DPSimplification.h"
#include "../util/ContourUtils.h"

namespace simp {

	/**
	 * Simplify the footprint of the layer.
	 *
	 * @param slices	slice images of the layer
	 * @param epsilon	simplification parameter
	 * @return			simplified footprint
	 */
	util::Polygon DPSimplification::simplify(const util::Polygon& polygon, float epsilon, float min_hole_ratio) {
		util::Polygon ans;
		util::approxPolyDP(polygon.contour.points, ans.contour.points, epsilon, true);
		if (ans.contour.points.size() < 3) throw "Invalid simplified polygon";

		double area = cv::contourArea(ans.contour.points);

		// simplify the hole as well
		for (auto& hole : polygon.holes) {
			if (cv::contourArea(hole.points) < area * min_hole_ratio) continue;

			try {
				util::Ring simplified_hole;
				util::approxPolyDP(hole.points, simplified_hole.points, epsilon, true);
				if (simplified_hole.size() >= 3 && util::withinPolygon(simplified_hole, ans.contour)) {
					ans.holes.push_back(simplified_hole);
				}
			}
			catch (...) {}
		}

		// The transfomration matrix should be same for the external contour and the internal holes
		// because for OpenCV simplification, transformation is just to trasnform from OpenCV image coordinates
		// to the world coordinate system.
		ans.mat = ans.contour.mat;

		// ToDo:
		// Should we check if the holes are inside the contour?

		decomposePolygon(ans);

		return ans;
	}

	void DPSimplification::decomposePolygon(util::Polygon& polygon) {
		std::vector<std::vector<cv::Point2f>> polygons;

		if (polygon.holes.size() == 0) {
			polygons = util::tessellate(polygon.contour);
		}
		else {
			polygons = util::tessellate(polygon.contour, polygon.holes);
		}

		polygon.primitive_shapes.clear();

		for (int i = 0; i < polygons.size(); i++) {
			util::clockwise(polygons[i]);

			for (int j = 1; j < polygons[i].size() - 1; j++) {
				boost::shared_ptr<util::PrimitiveTriangle> pol = boost::shared_ptr<util::PrimitiveTriangle>(new util::PrimitiveTriangle(polygon.mat));
				pol->points.push_back(cv::Point2f(polygons[i][0].x, polygons[i][0].y));
				pol->points.push_back(cv::Point2f(polygons[i][j].x, polygons[i][j].y));
				pol->points.push_back(cv::Point2f(polygons[i][j + 1].x, polygons[i][j + 1].y));
				polygon.primitive_shapes.push_back(pol);
			}
		}
	}

}