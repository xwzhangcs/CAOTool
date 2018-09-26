#include "EfficientRansacSimplification.h"
#include "../util/ContourUtils.h"
#include <boost/math/distributions/normal.hpp>
#include <boost/geometry/geometries/segment.hpp> 
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/polygon/polygon.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace simp {

	/**
	* Simplify the footprint of the layer.
	*
	* @param polygon			contour polygon of the layer
	* @param epsilon			epsilon parameter for DP method
	* @param curve_threshold	maximum deviation of the point from the arc
	* @param angle_threshold	maximum angle deviation of the point from the axis aligned line
	* @param orientation		principle orientation of the contour in radian
	* @param min_hole_ratio		hole will be removed if its area ratio to the contour is less than this threshold
	* @return					simplified footprint
	*/
	util::Polygon EfficientRansacSimplification::simplify(const util::Polygon& polygon, const std::vector<double>& parameters, float orientation, float min_hole_ratio) {
		util::Polygon ans;
		// create a slice image from the input polygon
		cv::Rect bbox = util::boundingBox(polygon.contour.points);
		cv::Mat_<uchar> img;
		util::createImageFromPolygon(bbox.width, bbox.height, polygon, cv::Point2f(-bbox.x, -bbox.y), img);
		std::vector<util::Polygon> polygons = util::findContours(img, 40, false, true, false);
		if (polygons.size() == 0) throw "No building is found.";
		// debug
		/*{
			std::string img_name = "../data/" + std::to_string(rand() % 500) + ".png";
			cv::imwrite(img_name, img);
			std::cout << "img file name is " << img_name << std::endl;
		}*/
		// tranlsate (bbox.x, bbox.y)
		polygons[0].translate(bbox.x, bbox.y);

		// only consider the first polygon?
		generateContours(polygons[0], ans, parameters, orientation);
		if (ans.contour.size() < 3){
			throw "No building is found.";
		}
		return ans;
	}

	void EfficientRansacSimplification::generateContours(util::Polygon polygon, util::Polygon& ans, const std::vector<double>& parameters, float orientation) {
		// detect principal orientation
		float principal_orientation = util::estimate(polygon.contour.points);

		// use the principal orientation, +45, +90, +135 degrees as principal orientations
		std::vector<float> principal_orientations;
		for (int i = 0; i < 4; i++) {
			principal_orientations.push_back(principal_orientation + CV_PI * i / 4);
		}

		// read parameters of detection
		int curve_num_iterations = parameters[0];
		int curve_min_points = parameters[1];
		float curve_max_error_ratio_to_radius = parameters[2];
		float curve_cluster_epsilon = parameters[3];
		float curve_min_angle = parameters[4];
		float curve_min_radius = parameters[5];
		float curve_max_radius = parameters[6];

		int line_num_iterations = parameters[7];
		float line_min_points = parameters[8];
		float line_max_error = parameters[9];
		float line_cluster_epsilon = parameters[10];
		float line_min_length = parameters[11];
		float line_angle_threshold = parameters[12];

		float contour_max_error = parameters[13];
		float contour_angle_threshold = parameters[14];

		// detect circles and lines
		efficient_ransac::EfficientRANSAC er;
		std::vector<std::pair<int, std::shared_ptr<efficient_ransac::PrimitiveShape>>> shapes;
		std::vector<cv::Point2f> contour;
		std::vector<int> contourPointsType;
		std::vector<std::vector<cv::Point2f>> contour_holes;
		std::vector<std::vector<int>> contourPointsType_holes;
		bool bValid = false;
		bool bValidHoles = false;
		bool bContainCurve = false;
		if (polygon.contour.size() >= 100) {
			// relative parameters
			cv::Rect bbox = cv::boundingRect(polygon.contour.points);
			line_min_points = line_min_points * polygon.contour.size();
			line_cluster_epsilon = line_cluster_epsilon * polygon.contour.size();
			if (line_cluster_epsilon <= 3)
				line_cluster_epsilon = 3.0f;
			line_min_length = line_min_length * sqrt(bbox.width * bbox.width + bbox.height * bbox.height);
			//std::cout << "line_min_points is " << line_min_points << std::endl;
			shapes = er.detect(polygon.contour.points, curve_num_iterations, curve_min_points, curve_max_error_ratio_to_radius, curve_cluster_epsilon, curve_min_angle, curve_min_radius, curve_max_radius, line_num_iterations, line_min_points, line_max_error, line_cluster_epsilon, line_min_length, line_angle_threshold, principal_orientations);
			//std::cout << "shapes is " << shapes.size() << std::endl;
			if (shapes.size() > 0){
				std::sort(shapes.begin(), shapes.end());
				//std::cout << "Before shapes size is " << shapes.size() << std::endl;
				ContourGenerator::generate(polygon, shapes, contour, contourPointsType, contour_max_error, contour_angle_threshold);
				// check whether it's a simple contour
				while (contour.size() > 0 && !util::isSimple(contour) && shapes.size() > 2) {
					// remove the shortest line segment
					int best_line_index = -1;
					float best_line_length = std::numeric_limits<float>::max();
					for (int j = 0; j < shapes.size(); j++){
						if (efficient_ransac::Circle* circle = dynamic_cast<efficient_ransac::Circle*>(shapes[j].second.get())) {
							continue;
						}
						else{
							float length = cv::norm(shapes[j].second->startPoint() - shapes[j].second->endPoint());
							if (length < best_line_length){
								best_line_length = length;
								best_line_index = j;
							}
						}
					}
					if (best_line_index != -1){
						shapes.erase(shapes.begin() + best_line_index);
						contour.clear();
						contourPointsType.clear();
						ContourGenerator::generate(polygon, shapes, contour, contourPointsType, contour_max_error, contour_angle_threshold);
						//std::cout << "remove shortest line segment successfully.." << std::endl;
					}
					else{
						std::cout << "failure.." << std::endl;
						break;
					}
				}
				//std::cout << "After shapes size is " << shapes.size() << std::endl;
				if (contour.size() > 0 && util::isSimple(contour))
					bValid = true;
			}
		}
		if (!bValid){
			contour.clear();
			contourPointsType.clear();
			float epsilon = 4.0f;
			cv::approxPolyDP(polygon.contour.points, contour, epsilon, true);
			if (contour.size() < 3) {
				// If the simplification makes the polygon a line, gradually decrease the epsilon 
				// until it becomes a polygon with at least 3 vertices.
				float epsilon2 = epsilon - 2;
				while (epsilon2 >= 0 && contour.size() < 3) {
					contour.clear();
					cv::approxPolyDP(polygon.contour.points, contour, epsilon2, true);
					//util::approxPolyDP(polygon.contour.points, contour, epsilon2, true);
					epsilon2 -= 2;
				}
				if (contour.size() < 3) {
					contour = polygon.contour.points;
				}
			}
			for (int j = 0; j < contour.size(); j++){
				contourPointsType.push_back(0);
			}
		}

		// holes
		if (polygon.holes.size() > 0){
			contour_holes.resize(polygon.holes.size());
			contourPointsType_holes.resize(polygon.holes.size());
			bool bValid_hole = false; 
			for (int holeId = 0; holeId < polygon.holes.size(); holeId++){
				bValid_hole = false;
				std::vector<cv::Point2f> one_hole;
				one_hole.resize(polygon.holes[holeId].size());
				line_min_points = parameters[8];
				line_cluster_epsilon = parameters[10];
				line_min_length = parameters[11];
				std::vector<std::pair<int, std::shared_ptr<efficient_ransac::PrimitiveShape>>> shapes_hole;
				util::Polygon hole_polygon;
				for (int k = 0; k < polygon.holes[holeId].size(); k++)
					one_hole[k] = polygon.holes[holeId][k];
				if (one_hole.size() > 100 && cv::contourArea(one_hole) > 100){
					hole_polygon.contour.points = one_hole;
					//{
					//	cv::Rect bbox = util::boundingBox(hole_polygon.contour.points);
					//	//cv::Rect bbox = util::boundingBox(building->presentativeContours[0].contour.points);
					//	cv::Mat_<uchar> img;
					//	util::createImageFromPolygon(bbox.width, bbox.height, hole_polygon, cv::Point2f(-bbox.x, -bbox.y), img);
					//	cv::imwrite("../data/hole.png", img);
					//}
					cv::Rect bbox = cv::boundingRect(hole_polygon.contour.points);
					line_min_points = line_min_points * hole_polygon.contour.size();
					line_cluster_epsilon = line_cluster_epsilon * hole_polygon.contour.size();
					if (line_cluster_epsilon < 3)
						line_cluster_epsilon = 3.0f;
					line_min_length = line_min_length * sqrt(bbox.width * bbox.width + bbox.height * bbox.height);
					shapes_hole = er.detect(hole_polygon.contour.points, curve_num_iterations, curve_min_points, curve_max_error_ratio_to_radius, curve_cluster_epsilon, curve_min_angle, curve_min_radius, curve_max_radius, line_num_iterations, line_min_points, line_max_error, line_cluster_epsilon, line_min_length, line_angle_threshold, principal_orientations);
					if (shapes_hole.size() > 0){
						std::sort(shapes_hole.begin(), shapes_hole.end());
						ContourGenerator::generate(hole_polygon, shapes_hole, contour_holes[holeId], contourPointsType_holes[holeId], contour_max_error, contour_angle_threshold);
						if (contour_holes[holeId].size() > 0){
							bValid_hole = true;
							bValidHoles = true;
						}
					}
				}

				if (!bValid_hole){
					contour_holes[holeId].clear();
					contourPointsType_holes[holeId].clear();				
				}
			}
		}
		ans.contour.points = contour;
		ans.contour.pointsType = contourPointsType;
		if (bValidHoles){
			for (int i = 0; i < contour_holes.size(); i++){
				if (contour_holes[i].size() > 0){
					ans.holes.push_back(contour_holes[i]);
				}
			}
		}
		ans.mat = ans.contour.mat;
		std::vector<std::vector<cv::Point2f>> contours;
		if (bValidHoles)
			contours = util::tessellate(ans.contour, ans.holes);
		else
			contours = util::tessellate(ans.contour);

		for (int i = 0; i < contours.size(); i++) {
			util::clockwise(contours[i]);

			for (int j = 1; j < contours[i].size() - 1; j++) {
				boost::shared_ptr<util::PrimitiveTriangle> pol = boost::shared_ptr<util::PrimitiveTriangle>(new util::PrimitiveTriangle(ans.mat));
				pol->points.push_back(cv::Point2f(contours[i][0].x, contours[i][0].y));
				pol->points.push_back(cv::Point2f(contours[i][j].x, contours[i][j].y));
				pol->points.push_back(cv::Point2f(contours[i][j + 1].x, contours[i][j + 1].y));
				ans.primitive_shapes.push_back(pol);
			}
		}
	}

	/**
	* Helper function to extract contours from the input image.
	* The image has to be of type CV_8U, and has values either 0 or 255.
	* Note that the input image is not modified by this function.
	*/
	std::vector<util::Polygon> EfficientRansacSimplification::findContours(const cv::Mat_<uchar>& img) {
		std::vector<util::Polygon> ans;

		// resize x2
		cv::Mat_<uchar> img2;
		cv::resize(img, img2, cv::Size(img.cols * 2, img.rows * 2), 0, 0, cv::INTER_NEAREST);

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
				polygon.contour[j] = cv::Point2f(contours[i][j].x * 0.5, contours[i][j].y * 0.5);
			}

			if (polygon.contour.size() >= 3) {
				// obtain all the holes inside this contour
				int hole_id = hierarchy[i][2];
				while (hole_id != -1) {
					util::Ring hole;
					hole.resize(contours[hole_id].size());
					for (int j = 0; j < contours[hole_id].size(); j++) {
						hole[j] = cv::Point2f(contours[hole_id][j].x * 0.5, contours[hole_id][j].y * 0.5);
					}
					polygon.holes.push_back(hole);
					hole_id = hierarchy[hole_id][0];
				}

				ans.push_back(polygon);
			}
		}

		return ans;
	}
}