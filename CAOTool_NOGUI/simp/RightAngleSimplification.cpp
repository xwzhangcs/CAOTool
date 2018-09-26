#include "RightAngleSimplification.h"
#include "../util/ContourUtils.h"

namespace simp {

	/**
	* Simplify the footprint of the layer.
	*
	* @param polygon		contour polygon of the layer
	* @param resolution		simplification is performed based on the resolution
	* @param orientation	principle orientation of the contour in radian
	* @param min_hole_ratio	hole will be removed if its area ratio to the contour is less than this threshold
	* @return				simplified footprint
	*/
	util::Polygon RightAngleSimplification::simplify(const util::Polygon& polygon, int resolution, float orientation, float min_hole_ratio, bool optimization) {
		util::Polygon ans;

		std::tuple<float, int, int> best_mat = simplifyContour(polygon.contour, ans.contour, resolution, orientation, optimization);
		float angle = std::get<0>(best_mat);
		float dx = std::get<1>(best_mat);
		float dy = std::get<2>(best_mat);
		if (ans.contour.size() < 3) throw "Invalid contour. #vertices is less than 3.";

		double area = cv::contourArea(ans.contour.points);

		// simplify the hole as well
		for (auto& hole : polygon.holes) {
			if (cv::contourArea(hole.points) < area * min_hole_ratio) continue;

			try {
				util::Ring simplified_hole;
				simplifyContour(hole, simplified_hole, resolution, angle, dx, dy, true, optimization);
				if (simplified_hole.size() >= 3 && util::withinPolygon(simplified_hole, ans.contour)) {
					ans.holes.push_back(simplified_hole);
				}
			}
			catch (...) {
			}
		}

		// The transfomration matrix should be same for the external contour and the internal holes
		ans.mat = ans.contour.mat;

		// ToDo:
		// Should we check if the holes are inside the contour?

		decomposePolygon(ans);

		return ans;
	}

	/**
	* Simplify and regularize a polygon
	*
	* @param contour		input contour polygon
	* @param result		output regularized polygon
	* @param resolution	resolution which defines how much simplified
	* @return				best angle, dx, and dy that yiled the resulting simplified polygon
	*/
	std::tuple<float, int, int> RightAngleSimplification::simplifyContour(const util::Ring& contour, util::Ring& result, int resolution, float orientation, bool optimization) {
		result.clear();

		double min_cost = std::numeric_limits<double>::max();
		float best_angle;
		int best_dx;
		int best_dy;

		int step_size = std::max(1, resolution / 10);
		//for (int angle_id = 0; angle_id < 2; angle_id++) {
		for (int angle_id = 0; angle_id < 1; angle_id++) {
			float angle = orientation + angle_id * 45.0 / 180.0 * CV_PI;

			for (int dx = 0; dx < resolution; dx += step_size) {
				for (int dy = 0; dy < resolution; dy += step_size) {
					util::Ring simplified_contour;
					try {
						double cost = simplifyContour(contour, simplified_contour, resolution, angle, dx, dy, false, optimization);

						if (cost < min_cost) {
							min_cost = cost;
							best_angle = angle;
							best_dx = dx;
							best_dy = dy;

							result = simplified_contour;
						}
					}
					catch (...) {
					}
				}
			}
		}

		if (min_cost == std::numeric_limits<double>::max()) throw "No simplified polygon was found.";

		// refine the simplified contour
		simplifyContour(contour, result, resolution, best_angle, best_dx, best_dy, true, optimization);

		return std::make_tuple(best_angle, best_dx, best_dy);
	}

	/**
	* Simplify and regularize a polygon using the specified angle, dx, and dy.
	*
	* @param contour		input contour polygon
	* @param result			output regularized polygon
	* @param resolution		resolution which defines how much simplified
	* @return				best cost
	*/
	double RightAngleSimplification::simplifyContour(const util::Ring& contour, util::Ring& result, int resolution, float theta, int dx, int dy, bool refine, bool vertex_refinement) {
		// create a transformation matrix
		cv::Mat_<float> M = (cv::Mat_<float>(3, 3) << cos(theta), -sin(theta), dx, sin(theta), cos(theta), dy, 0, 0, 1);

		// create inverse transformation matrix
		cv::Mat_<float> invM = M.inv();

		// transform the polygon
		std::vector<cv::Point2f> aa_contour(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << contour[i].x, contour[i].y, 1);
			cv::Mat_<float> p2 = M * p;
			aa_contour[i] = cv::Point2f(p2(0, 0), p2(1, 0));
		}

		// create the integer coordinates of the polygon
		std::vector<cv::Point> aa_contour_int(aa_contour.size());
		for (int i = 0; i < aa_contour.size(); i++) {
			aa_contour_int[i] = cv::Point(std::round(aa_contour[i].x), std::round(aa_contour[i].y));
		}

		// simplify the contour a little
		std::vector<cv::Point2f> aa_contour_smoothed;
		util::approxPolyDP(aa_contour, aa_contour_smoothed, resolution, true);
		/*if (aa_contour_smoothed.size() > 3 && util::calculateIOU(aa_contour, aa_contour_smoothed) > 0.8) {
			aa_contour = aa_contour_smoothed;
		}*/

		// scale down the polygon based on the resolution
		std::vector<cv::Point> small_aa_polygon(aa_contour.size());
		for (int i = 0; i < aa_contour.size(); i++) {
			small_aa_polygon[i] = cv::Point(std::round(aa_contour[i].x / resolution), std::round(aa_contour[i].y / resolution));
		}

		// remove small spikes
		/*
		for (int i = (int)small_aa_polygon.size() - 2; i >= 0; i--) {
			int prev = (i - 1 + small_aa_polygon.size()) % small_aa_polygon.size();
			int next = (i + 1) % small_aa_polygon.size();
			if (small_aa_polygon[prev] == small_aa_polygon[next]) {
				small_aa_polygon.erase(small_aa_polygon.begin() + i, small_aa_polygon.begin() + i + 2);
				i--;
			}
		}
		*/

		// calculate the bounding box
		cv::Rect bbox = util::boundingBox(small_aa_polygon);
		if (bbox.width <= 1 && bbox.height <= 1) throw "Too small polygon.";

		cv::Mat_<uchar> img;// = cv::Mat_<uchar>::zeros(bbox.height, bbox.width);
		util::createImageFromContour(bbox.width + 2, bbox.height + 2, small_aa_polygon, cv::Point(1 - bbox.x, 1 - bbox.y), img, true);

		// clean the contour by removing small bumps
		for (int r = 1; r < img.rows - 1; r++) {
			for (int c = 1; c < img.cols - 1; c++) {
				if (img(r, c) == 255) {
					if ((img(r - 1, c) == 0 && img(r, c - 1) == 0 && img(r, c + 1) == 0 && img(r + 1, c) == 255)
						|| (img(r - 1, c) == 0 && img(r, c - 1) == 0 && img(r + 1, c) == 0 && img(r, c + 1) == 255)
						|| (img(r, c - 1) == 0 && img(r + 1, c) == 0 && img(r, c + 1) == 0 && img(r - 1, c) == 255)
						|| (img(r - 1, c) == 0 && img(r, c + 1) == 0 && img(r + 1, c) == 0 && img(r, c - 1) == 255)) {
						img(r, c) = 0;
					}
				}
				else {
					if ((img(r - 1, c) == 0 && img(r, c - 1) == 255 && img(r, c + 1) == 255 && img(r + 1, c) == 255 && img(r + 1, c - 1) == 255 && img(r + 1, c + 1) == 255)
						|| (img(r, c - 1) == 0 && img(r - 1, c) == 255 && img(r, c + 1) == 255 && img(r + 1, c) == 255 && img(r - 1, c + 1) == 255 && img(r + 1, c + 1) == 255)
						|| (img(r + 1, c) == 0 && img(r - 1, c) == 255 && img(r, c - 1) == 255 && img(r, c + 1) == 255 && img(r - 1, c - 1) == 255 && img(r - 1, c + 1) == 255)
						|| (img(r, c + 1) == 0 && img(r - 1, c) == 255 && img(r, c - 1) == 255 && img(r + 1, c) == 255 && img(r - 1, c - 1) == 255 && img(r + 1, c - 1) == 255)) {
						img(r, c) = 255;
					}
				}
			}
		}

		std::vector<util::Polygon> polygons = findContours(img);
		if (polygons.size() == 0) throw "No contour is found.";
		if (polygons.size() == 2) throw "Multiple contours are found.";

		// offset back and scale up the simplified scale-down polygon
		std::vector<cv::Point2f> simplified_aa_contour(polygons[0].contour.size());
		for (int i = 0; i < polygons[0].contour.size(); i++) {
			simplified_aa_contour[i] = cv::Point2f((polygons[0].contour[i].x + bbox.x - 1) * resolution, (polygons[0].contour[i].y + bbox.y - 1) * resolution);
		}
		
		if (refine) {
			std::vector<cv::Point> simplified_aa_contour_int(simplified_aa_contour.size());
			for (int i = 0; i < simplified_aa_contour.size(); i++) {
				simplified_aa_contour_int[i] = cv::Point(std::round(simplified_aa_contour[i].x), std::round(simplified_aa_contour[i].y));
			}
			double cost;
			if (vertex_refinement) {
				cost = 1.0 / (0.01 + optimizeVertices(aa_contour_int, simplified_aa_contour_int));
			}
			else {
				cost = 1.0 / (0.01 + optimizeBBox(aa_contour_int, simplified_aa_contour_int));
			}

			// generate a simplified contour
			if (util::isSimple(simplified_aa_contour_int)) {
				simplified_aa_contour.resize(simplified_aa_contour_int.size());
				for (int i = 0; i < simplified_aa_contour_int.size(); i++) {
					simplified_aa_contour[i] = cv::Point2f(simplified_aa_contour_int[i].x, simplified_aa_contour_int[i].y);
				}
			}
			else {
				// remove self-intersections
				bbox = util::boundingBox(simplified_aa_contour);
				util::createImageFromContour(bbox.width, bbox.height, simplified_aa_contour_int, cv::Point(-bbox.x, -bbox.y), img, false);
				polygons = util::findContours(img, false);
				if (polygons.size() == 0) throw "No valid contour is generated.";
				for (int i = 0; i < polygons[0].contour.size(); i++) {
					polygons[0].contour[i] += cv::Point2f(bbox.x, bbox.y);
				}

				simplified_aa_contour = polygons[0].contour.points;
			}
		}

		// calculate the cost
		double cost = 1.0 / (0.01 + util::calculateIOU(aa_contour, simplified_aa_contour));		
		cost += simplified_aa_contour.size() * 0.2;

		// transform back the simplified contour
		util::Ring simplified_contour = simplified_aa_contour;
		simplified_contour.mat = invM;

		result = simplified_contour;

		return cost;
	}

	/**
	* Optimize the simplified contour such that it best fits to the input contour.
	* Return the IOU of the input contour and the optimized simplified contour.
	*
	* @param contour				input contour
	* @param simplified_contour		simplified contour
	* @return						intersection over union (IOU)
	*/
	double RightAngleSimplification::optimizeVertices(const std::vector<cv::Point>& contour, std::vector<cv::Point>& simplified_contour) {
		// calculate the bounding box
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();
		int max_y = -std::numeric_limits<int>::max();
		for (int i = 0; i < contour.size(); i++) {
			min_x = std::min(min_x, contour[i].x);
			max_x = std::max(max_x, contour[i].x);
			min_y = std::min(min_y, contour[i].y);
			max_y = std::max(max_y, contour[i].y);
		}
		for (int i = 0; i < simplified_contour.size(); i++) {
			min_x = std::min(min_x, simplified_contour[i].x);
			max_x = std::max(max_x, simplified_contour[i].x);
			min_y = std::min(min_y, simplified_contour[i].y);
			max_y = std::max(max_y, simplified_contour[i].y);
		}

		// create the image of the input contour
		cv::Mat_<uchar> img;
		util::createImageFromContour(max_x - min_x + 1, max_y - min_y + 1, contour, cv::Point(-min_x, -min_y), img);

		// list up the parameters
		std::map<int, int> x_map;
		std::map<int, int> y_map;
		for (int i = 0; i < simplified_contour.size(); i++) {
			if (x_map.find(simplified_contour[i].x) == x_map.end()) {
				x_map[simplified_contour[i].x] = simplified_contour[i].x;
			}
			if (y_map.find(simplified_contour[i].y) == y_map.end()) {
				y_map[simplified_contour[i].y] = simplified_contour[i].y;
			}
		}

		// optimize the parameters
		double best_score = 0;
		std::map<int, int> best_x_map;
		std::map<int, int> best_y_map;

		int cnt = 0;
		for (int iter = 0; iter < 3000; iter++) {
			bool updated = false;

			auto prev_it = x_map.end();
			auto next_it = x_map.begin();
			next_it++;
			for (auto it = x_map.begin(); it != x_map.end(); it++) {
				std::map<int, int> prop_x_map = x_map;
				prop_x_map[it->first]++;
				if ((next_it != x_map.end() && prop_x_map[it->first] <= prop_x_map[next_it->first]) || (next_it == x_map.end() && prop_x_map[it->first] <= max_x)) {
					std::vector<cv::Point> proposed_contour = proposedContour(simplified_contour, prop_x_map, y_map);
					cv::Mat_<uchar> img2;
					util::createImageFromContour(max_x - min_x + 1, max_y - min_y + 1, proposed_contour, cv::Point(-min_x, -min_y), img2);
					double score = util::calculateIOU(img, img2);
					if (score > best_score) {
						best_score = score;
						best_x_map = prop_x_map;
						best_y_map = y_map;
						updated = true;
					}
				}

				prop_x_map = x_map;
				prop_x_map[it->first]--;
				if ((prev_it != x_map.end() && prop_x_map[it->first] >= prop_x_map[prev_it->first]) || (prev_it == x_map.end() && prop_x_map[it->first] >= min_x)) {
					std::vector<cv::Point> proposed_contour = proposedContour(simplified_contour, prop_x_map, y_map);
					cv::Mat_<uchar> img2;
					util::createImageFromContour(max_x - min_x + 1, max_y - min_y + 1, proposed_contour, cv::Point(-min_x, -min_y), img2);
					double score = util::calculateIOU(img, img2);
					if (score > best_score) {
						best_score = score;
						best_x_map = prop_x_map;
						best_y_map = y_map;
						updated = true;
					}
				}

				prev_it = it;
				if (next_it != x_map.end())	next_it++;
			}

			prev_it = y_map.end();
			next_it = y_map.begin();
			next_it++;
			for (auto it = y_map.begin(); it != y_map.end(); it++) {
				std::map<int, int> prop_y_map = y_map;
				prop_y_map[it->first]++;
				if ((next_it != y_map.end() && prop_y_map[it->first] <= prop_y_map[next_it->first]) || (next_it == y_map.end() && prop_y_map[it->first] <= max_y)) {
					std::vector<cv::Point> proposed_contour = proposedContour(simplified_contour, x_map, prop_y_map);
					cv::Mat_<uchar> img2;
					util::createImageFromContour(max_x - min_x + 1, max_y - min_y + 1, proposed_contour, cv::Point(-min_x, -min_y), img2);
					double score = util::calculateIOU(img, img2);
					if (score > best_score) {
						best_score = score;
						best_x_map = x_map;
						best_y_map = prop_y_map;
						updated = true;
					}
				}

				prop_y_map = y_map;
				prop_y_map[it->first]--;
				if ((prev_it != y_map.end() && prop_y_map[it->first] >= prop_y_map[prev_it->first]) || (prev_it == y_map.end() && prop_y_map[it->first] >= min_y)) {
					std::vector<cv::Point> proposed_contour = proposedContour(simplified_contour, x_map, prop_y_map);
					cv::Mat_<uchar> img2;
					util::createImageFromContour(max_x - min_x + 1, max_y - min_y + 1, proposed_contour, cv::Point(-min_x, -min_y), img2);
					double score = util::calculateIOU(img, img2);
					if (score > best_score) {
						best_score = score;
						best_x_map = x_map;
						best_y_map = prop_y_map;
						updated = true;
					}
				}

				prev_it = it;
				if (next_it != y_map.end())	next_it++;
			}

			// if no update, stop the optimization
			if (!updated) break;

			x_map = best_x_map;
			y_map = best_y_map;
		}

		simplified_contour = proposedContour(simplified_contour, x_map, y_map);

		// remove redundant points
		for (int i = simplified_contour.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + simplified_contour.size()) % simplified_contour.size();
			if (simplified_contour[i] == simplified_contour[prev]) simplified_contour.erase(simplified_contour.begin() + i);
		}

		// remove colinear points
		for (int i = simplified_contour.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + simplified_contour.size()) % simplified_contour.size();
			int next = (i + 1) % simplified_contour.size();
			if (std::abs(util::crossProduct(simplified_contour[i] - simplified_contour[prev], simplified_contour[next] - simplified_contour[i])) < 0.00001) simplified_contour.erase(simplified_contour.begin() + i);
		}

		return best_score;
	}

	std::vector<cv::Point> RightAngleSimplification::proposedContour(const std::vector<cv::Point>& contour, std::map<int, int>& x_map, std::map<int, int>& y_map) {
		std::vector<cv::Point> prop_contour(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			prop_contour[i] = cv::Point(x_map[contour[i].x], y_map[contour[i].y]);
		}
		return prop_contour;
	}

	/**
	 * Optimize the simplified contour such that it best fits to the input contour.
	 * Return the IOU of the input contour and the optimized simplified contour.
	 *
	 * @param contour				input contour
	 * @param simplified_contour		simplified contour
	 * @return						intersection over union (IOU)
	 */
	double RightAngleSimplification::optimizeBBox(const std::vector<cv::Point>& contour, std::vector<cv::Point>& simplified_contour) {
		// calculate the bounding box
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();
		int max_y = -std::numeric_limits<int>::max();
		for (int i = 0; i < contour.size(); i++) {
			min_x = std::min(min_x, contour[i].x);
			max_x = std::max(max_x, contour[i].x);
			min_y = std::min(min_y, contour[i].y);
			max_y = std::max(max_y, contour[i].y);
		}
		for (int i = 0; i < simplified_contour.size(); i++) {
			min_x = std::min(min_x, simplified_contour[i].x);
			max_x = std::max(max_x, simplified_contour[i].x);
			min_y = std::min(min_y, simplified_contour[i].y);
			max_y = std::max(max_y, simplified_contour[i].y);
		}

		int image_width = max_x - min_x + 200;
		int image_height = max_y - min_y + 200;
		min_x -= 100;
		min_y -= 100;

		// create the image of the input contour
		cv::Mat_<uchar> img;
		util::createImageFromContour(image_width, image_height, contour, cv::Point(-min_x, -min_y), img);

		// calculate the bounding box of the simplified contour
		int x1 = std::numeric_limits<int>::max();
		int x2 = -std::numeric_limits<int>::max();
		int y1 = std::numeric_limits<int>::max();
		int y2 = -std::numeric_limits<int>::max();
		for (int i = 0; i < contour.size(); i++) {
			x1 = std::min(x1, contour[i].x);
			x2 = std::max(x2, contour[i].x);
			y1 = std::min(y1, contour[i].y);
			y2 = std::max(y2, contour[i].y);
		}

		// optimize the parameters
		double best_score = 0;
		int best_x1 = x1;
		int best_x2 = x2;
		int best_y1 = y1;
		int best_y2 = y2;
		{
			std::vector<cv::Point> current_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, x1, x2, y1, y2);
			cv::Mat_<uchar> img2;
			util::createImageFromContour(image_width, image_height, current_contour, cv::Point(-min_x, -min_y), img2);
			best_score = util::calculateIOU(img, img2);
		}

		int cur_x1 = x1;
		int cur_x2 = x2;
		int cur_y1 = y1;
		int cur_y2 = y2;

		int cnt = 0;
		for (int iter = 0; iter < 300; iter++) {
			bool updated = false;

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1 - 1, cur_x2, cur_y1, cur_y2);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1 - 1;
					best_x2 = cur_x2;
					best_y1 = cur_y1;
					best_y2 = cur_y2;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1 + 1, cur_x2, cur_y1, cur_y2);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1 + 1;
					best_x2 = cur_x2;
					best_y1 = cur_y1;
					best_y2 = cur_y2;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1, cur_x2 - 1, cur_y1, cur_y2);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1;
					best_x2 = cur_x2 - 1;
					best_y1 = cur_y1;
					best_y2 = cur_y2;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1, cur_x2 + 1, cur_y1, cur_y2);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1;
					best_x2 = cur_x2 + 1;
					best_y1 = cur_y1;
					best_y2 = cur_y2;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1, cur_x2, cur_y1 - 1, cur_y2);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1;
					best_x2 = cur_x2;
					best_y1 = cur_y1 - 1;
					best_y2 = cur_y2;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1, cur_x2, cur_y1 + 1, cur_y2);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1;
					best_x2 = cur_x2;
					best_y1 = cur_y1 + 1;
					best_y2 = cur_y2;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1, cur_x2, cur_y1, cur_y2 - 1);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1;
					best_x2 = cur_x2;
					best_y1 = cur_y1;
					best_y2 = cur_y2 - 1;
					updated = true;
				}
			}

			{
				std::vector<cv::Point> proposed_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, cur_x1, cur_x2, cur_y1, cur_y2 + 1);
				cv::Mat_<uchar> img2;
				util::createImageFromContour(image_width, image_height, proposed_contour, cv::Point(-min_x, -min_y), img2);
				double score = util::calculateIOU(img, img2);
				if (score > best_score) {
					best_score = score;
					best_x1 = cur_x1;
					best_x2 = cur_x2;
					best_y1 = cur_y1;
					best_y2 = cur_y2 + 1;
					updated = true;
				}
			}

			// if no update, stop the optimization
			if (!updated) break;

			cur_x1 = best_x1;
			cur_x2 = best_x2;
			cur_y1 = best_y1;
			cur_y2 = best_y2;
		}

		simplified_contour = proposedBBox(simplified_contour, x1, x2, y1, y2, best_x1, best_x2, best_y1, best_y2);
		return best_score;
	}

	std::vector<cv::Point> RightAngleSimplification::proposedBBox(const std::vector<cv::Point>& contour, int x1, int x2, int y1, int y2, int new_x1, int new_x2, int new_y1, int new_y2) {
		std::vector<cv::Point> prop_contour(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			prop_contour[i] = cv::Point(std::round((float)(contour[i].x - x1) / (x2 - x1) * (new_x2 - new_x1) + new_x1), std::round((float)(contour[i].y - y1) / (y2 - y1) * (new_y2 - new_y1) + new_y1));
		}
		return prop_contour;
	}

	void RightAngleSimplification::decomposePolygon(util::Polygon& polygon) {
		// list up all xy coordinates
		std::map<float, bool> x_map;
		std::map<float, bool> y_map;
		for (int i = 0; i < polygon.contour.size(); i++) {
			x_map[polygon.contour[i].x] = true;
			y_map[polygon.contour[i].y] = true;
		}
		for (int i = 0; i < polygon.holes.size(); i++) {
			for (int j = 0; j < polygon.holes[i].size(); j++) {
				x_map[polygon.holes[i][j].x] = true;
				y_map[polygon.holes[i][j].y] = true;
			}
		}

		// create vector of xy coordinates
		std::vector<float> x_coords;
		for (auto it = x_map.begin(); it != x_map.end(); it++) {
			x_coords.push_back(it->first);
		}
		std::vector<float> y_coords;
		for (auto it = y_map.begin(); it != y_map.end(); it++) {
			y_coords.push_back(it->first);
		}

		std::vector<std::vector<bool>> grid(y_coords.size() - 1, std::vector<bool>(x_coords.size() - 1, false));
		int cell_count = 0;
		for (int i = 0; i < x_coords.size() - 1; i++) {
			for (int j = 0; j < y_coords.size() - 1; j++) {
				if (util::withinPolygon(cv::Point2f((x_coords[i] + x_coords[i + 1]) * 0.5, (y_coords[j] + y_coords[j + 1]) * 0.5), polygon)) {
					grid[j][i] = true;
					cell_count++;
				}
			}
		}

		polygon.primitive_shapes.clear();

		while (cell_count > 0) {
			int x, y, width, height;
			findMaximumRectangle(grid, x_coords, y_coords, x, y, width, height);
			
			// update grid
			for (int r = y; r < y + height; r++) {
				for (int c = x; c < x + width; c++) {
					grid[r][c] = false;
				}
			}
			cell_count -= width * height;

			polygon.primitive_shapes.push_back(boost::shared_ptr<util::PrimitiveShape>(new util::PrimitiveRectangle(polygon.mat, cv::Point2f(x_coords[x], y_coords[y]), cv::Point2f(x_coords[x + width], y_coords[y + height]))));
		}
	}

	/**
	 * Extract the contour from the image.
	 * The edge should be axis aligned or 45 degree oriented.
	 */
	std::vector<util::Polygon> RightAngleSimplification::findContours(const cv::Mat_<uchar>& img) {
		std::vector<util::Polygon> ans;

		cv::Mat_<uchar> img2 = img.clone();
		while (true) {
			bool updated = false;
			for (int r = 0; r < img.rows - 1; r++) {
				for (int c = 0; c < img.cols - 1; c++) {
					if (img2(r, c) == 255 && img2(r + 1, c + 1) == 255 && img2(r + 1, c) == 0 && img2(r, c + 1) == 0) {
						updated = true;
						img2(r + 1, c) = 255;
					}
					else if (img2(r, c) == 0 && img2(r + 1, c + 1) == 0 && img2(r + 1, c) == 255 && img2(r, c + 1) == 255) {
						updated = true;
						img2(r, c) = 255;
					}
				}
			}
			if (!updated) break;
		}

		// add padding
		cv::Mat_<uchar> padded = cv::Mat_<uchar>::zeros(img2.rows + 1, img2.cols + 1);
		img2.copyTo(padded(cv::Rect(0, 0, img2.cols, img2.rows)));

		// dilate image
		cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 1, 1, 0, 1, 1, 0, 0, 0, 0);
		cv::dilate(padded, padded, kernel);

		// extract contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(padded, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		for (int i = 0; i < hierarchy.size(); i++) {
			if (hierarchy[i][3] != -1) continue;
			if (contours[i].size() < 3) continue;

			util::Polygon polygon;
			polygon.contour.resize(contours[i].size());
			for (int j = 0; j < contours[i].size(); j++) {
				polygon.contour[j] = cv::Point2f(contours[i][j].x, contours[i][j].y);
			}
			polygon.contour = util::removeRedundantPoint(polygon.contour);
			refineContour(polygon.contour);

			if (polygon.contour.size() >= 3) {
				// obtain all the holes inside this contour
				int hole_id = hierarchy[i][2];
				while (hole_id != -1) {
					util::Ring hole;
					hole.resize(contours[hole_id].size());
					for (int j = 0; j < contours[hole_id].size(); j++) {
						hole[j] = cv::Point2f(contours[hole_id][j].x, contours[hole_id][j].y);
					}
					hole = util::removeRedundantPoint(hole);
					refineContour(hole);
					polygon.holes.push_back(hole);
					hole_id = hierarchy[hole_id][0];
				}

				ans.push_back(polygon);
			}
		}
		
		return ans;
	}

	void RightAngleSimplification::refineContour(util::Ring& polygon) {
		polygon.counterClockwise();

		util::Ring new_polygon;
		for (int i = 0; i < polygon.size(); i++) {
			new_polygon.push_back(polygon[i]);

			int next = (i + 1) % polygon.size();
			if (std::abs(polygon[i].x - polygon[next].x) == 1 && std::abs(polygon[i].y - polygon[next].y) == 1) {
				if (polygon[next].x - polygon[i].x == 1 && polygon[next].y - polygon[i].y == 1) {
					new_polygon.push_back(cv::Point2f(polygon[i].x, polygon[next].y));
				}
				else if (polygon[next].x - polygon[i].x == 1 && polygon[next].y - polygon[i].y == -1) {
					new_polygon.push_back(cv::Point2f(polygon[next].x, polygon[i].y));
				}
				else if (polygon[next].x - polygon[i].x == -1 && polygon[next].y - polygon[i].y == 1) {
					new_polygon.push_back(cv::Point2f(polygon[next].x, polygon[i].y));
				}
				else {
					new_polygon.push_back(cv::Point2f(polygon[i].x, polygon[next].y));
				}				
			}
		}
		polygon = new_polygon;

		// remove the redundant points just in case
		for (int i = polygon.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + polygon.size()) % polygon.size();
			if (polygon[i].x == polygon[prev].x && polygon[i].y == polygon[prev].y) polygon.points.erase(polygon.points.begin() + i);
		}

		// remove the collinear points
		for (int i = polygon.size() - 1; i >= 0; i--) {
			int prev = (i - 1 + polygon.size()) % polygon.size();
			int next = (i + 1) % polygon.size();

			if ((polygon[i].x - polygon[prev].x) * (polygon[next].y - polygon[i].y) == (polygon[i].y - polygon[prev].y) * (polygon[next].x - polygon[i].x)) {
				polygon.points.erase(polygon.points.begin() + i);
			}
		}
	}

	void RightAngleSimplification::findMaximumRectangle(const std::vector<std::vector<bool>>& grid, const std::vector<float>& x_coords, const std::vector<float>& y_coords, int& x, int& y, int& width, int& height) {
		float max_area = 0;
		
		for (int r = 0; r < grid.size(); r++) {
			for (int c = 0; c < grid[r].size(); c++) {

				for (int r2 = r; r2 < grid.size(); r2++) {
					if (!grid[r2][c]) break;

					for (int c2 = c; c2 < grid[r2].size(); c2++) {
						// check the validity
						bool valid = true;
						for (int r3 = r; r3 <= r2; r3++) {
							if (!grid[r3][c2]) {
								valid = false;
								break;
							}
						}

						if (!valid) break;

						float area = (x_coords[c2 + 1] - x_coords[c]) * (y_coords[r2 + 1] - y_coords[r]);
						if (area > max_area) {
							max_area = area;
							x = c;
							y = r;
							width = c2 - c + 1;
							height = r2 - r + 1;
						}
					}
				}
			}
		}
	}

}
