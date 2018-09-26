#include "CurveRightAngleSimplification.h"
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
	util::Polygon CurveRightAngleSimplification::simplify(const util::Polygon& polygon, float epsilon, float curve_threshold, float angle_threshold, float orientation, float min_hole_ratio) {
		util::Polygon ans;
		angle_threshold = angle_threshold * 180.0 / CV_PI;
		// create a slice image from the input polygon
		cv::Rect bbox = util::boundingBox(polygon.contour.points);
		cv::Mat_<uchar> img;
		util::createImageFromPolygon(bbox.width, bbox.height, polygon, cv::Point2f(-bbox.x, -bbox.y), img);
		
		std::vector<util::Polygon> polygons = findContours(img);
		if (polygons.size() == 0) throw "No building is found.";

		// tranlsate (bbox.x, bbox.y)
		polygons[0].translate(bbox.x, bbox.y);

		decomposePolygon(polygons[0], ans, epsilon, curve_threshold, angle_threshold, orientation);
		if (ans.contour.size() < 3){
			throw "No building is found.";
		}
		return ans;
	}

	void CurveRightAngleSimplification::decomposePolygon(util::Polygon input, util::Polygon& polygon, float epsilon, float curve_threshold, float angle_threshold, float orientation) {
		// check whether it's valid holes
		bool bValidHoles = false;
		if (input.holes.size() > 0){
			for (int holeId = 0; holeId < input.holes.size(); holeId++){
				std::vector<cv::Point2f> one_hole;
				std::vector<cv::Point2f> contour;
				contour.resize(input.holes[holeId].size());
				for (int k = 0; k < input.holes[holeId].size(); k++)
					contour[k] = input.holes[holeId][k];
				util::approxPolyDP(cv::Mat(contour), one_hole, epsilon, true);

				if (one_hole.size() >= 3)
					bValidHoles = true;
			}
		}
		if (input.holes.size() == 0 || !bValidHoles){
			std::vector<cv::Point2f> contour;
			contour.resize(input.contour.size());
			for (int i = 0; i < input.contour.size(); i++)
				contour[i] = input.contour[i];
			bool bContainCurve = false;
			util::Polygon output;
			if (contour.size() > 500){
				bContainCurve = approxContour(contour, output, epsilon, curve_threshold, angle_threshold, orientation);
				if (output.contour.size() <= 3)
					bContainCurve = false;
			}
			if (bContainCurve)
			{
				polygon.contour = output.contour;
				polygon.mat = polygon.contour.mat;
				polygon.primitive_shapes = output.primitive_shapes;
			}
			else{
				//cv::approxPolyDP(cv::Mat(contour), polygon.contour.points, epsilon, true);
				util::approxPolyDP(cv::Mat(contour), polygon.contour.points, epsilon, true);

				if (polygon.contour.points.size() < 3) {
					// If the simplification makes the polygon a line, gradually increase the epsilon 
					// until it becomes a polygon with at least 3 vertices.
					float epsilon2 = epsilon - 0.3;
					while (epsilon2 >= 0 && polygon.contour.points.size() < 3) {
						//cv::approxPolyDP(contour, polygon.contour.points, epsilon2, true);
						util::approxPolyDP(contour, polygon.contour.points, epsilon2, true);
						epsilon2 -= 0.3;
					}
					if (polygon.contour.points.size() < 3) {
						polygon.contour.points = contour;
					}
				}

				//rectify the contour
				std::vector<cv::Point2f> results_tmp;
				//get maximal direction angle
				// create a slice image from the input polygon
				cv::Rect bbox = util::boundingBox(polygon.contour.points);
				cv::Mat_<uchar> img;
				util::createImageFromPolygon(bbox.width, bbox.height, polygon, cv::Point2f(-bbox.x, -bbox.y), img);
				float angle = orientation * 180 / CV_PI;
				cv::Mat_<float> M;
				polygon.contour.points = transform_angle(polygon.contour.points, M, angle);
				//results_tmp = contour_rectify(polygon.contour.points, angle_threshold, epsilon);
				results_tmp = contour_rectify_no_curve(polygon.contour.points, angle_threshold, epsilon);
				// remove redundant_points
				polygon.contour.points = del_redundant_points(results_tmp);
				if (!util::isSimple(polygon.contour)){
					polygon.contour.points.clear();
					util::approxPolyDP(cv::Mat(results_tmp), polygon.contour.points, epsilon, true);
				}
				// create inverse transformation matrix
				polygon.contour.points = transform(polygon.contour.points, M.inv());


				polygon.mat = polygon.contour.mat;
				std::vector<std::vector<cv::Point2f>> contours;
				contours = util::tessellate(polygon.contour);

				for (int i = 0; i < contours.size(); i++) {
					util::clockwise(contours[i]);

					for (int j = 1; j < contours[i].size() - 1; j++) {
						boost::shared_ptr<util::PrimitiveTriangle> pol = boost::shared_ptr<util::PrimitiveTriangle>(new util::PrimitiveTriangle(polygon.mat));
						pol->points.push_back(cv::Point2f(contours[i][0].x, contours[i][0].y));
						pol->points.push_back(cv::Point2f(contours[i][j].x, contours[i][j].y));
						pol->points.push_back(cv::Point2f(contours[i][j + 1].x, contours[i][j + 1].y));
						polygon.primitive_shapes.push_back(pol);
					}
				}
					
			}
		}
		else{
			// outer contour
			std::vector<cv::Point2f> contour;
			contour.resize(input.contour.size());
			
			for (int i = 0; i < input.contour.size(); i++)
				contour[i] = input.contour[i];

			util::Polygon output;
			bool bContainCurve = false;
			if (contour.size() > 500){
				bContainCurve = approxContour(contour, output, epsilon, curve_threshold, angle_threshold, orientation);
				if (output.contour.size() <= 3)
					bContainCurve = false;
			}
			if (bContainCurve)
			{
				polygon.contour = output.contour;
			}
			else{
				//cv::approxPolyDP(cv::Mat(contour), polygon.contour.points, epsilon, true);
				util::approxPolyDP(cv::Mat(contour), polygon.contour.points, epsilon, true);

				if (polygon.contour.points.size() < 3) {
						// If the simplification makes the polygon a line, gradually increase the epsilon 
						// until it becomes a polygon with at least 3 vertices.
						float epsilon2 = epsilon - 0.3;
						while (epsilon2 >= 0 && polygon.contour.points.size() < 3) {
							//cv::approxPolyDP(contour, polygon.contour.points, epsilon2, true);
							util::approxPolyDP(contour, polygon.contour.points, epsilon2, true);
							epsilon2 -= 0.3;
						}
						if (polygon.contour.points.size() < 3) {
							polygon.contour.points = contour;
						}
				}

				//rectify the contour
				std::vector<cv::Point2f> results_tmp;
				//get maximal direction angle
				cv::Rect bbox = util::boundingBox(polygon.contour.points);
				cv::Mat_<uchar> img;
				util::createImageFromPolygon(bbox.width, bbox.height, polygon, cv::Point2f(-bbox.x, -bbox.y), img);
				float angle = orientation * 180 / CV_PI;
				cv::Mat_<float> M;
				polygon.contour.points = transform_angle(polygon.contour.points, M, angle);
				//results_tmp = contour_rectify(polygon.contour.points, angle_threshold, epsilon);
				results_tmp = contour_rectify_no_curve(polygon.contour.points, angle_threshold, epsilon);
				// remove redundant_points
				polygon.contour.points = del_redundant_points(results_tmp);
				if (!util::isSimple(polygon.contour)){
					polygon.contour.points.clear();
					util::approxPolyDP(cv::Mat(results_tmp), polygon.contour.points, epsilon, true);
				}
				// create inverse transformation matrix
				polygon.contour.points = transform(polygon.contour.points, M.inv());
			}
			// holes
			for (int holeId = 0; holeId < input.holes.size(); holeId++){
				util::Ring simplified_hole;
				std::vector<cv::Point2f> contour;
				contour.resize(input.holes[holeId].size());
				for (int k = 0; k < input.holes[holeId].size(); k++)
					contour[k] = input.holes[holeId][k];

				util::Polygon output;
				bool bContainCurve = false;
				if (contour.size() > 500){
					bContainCurve = approxContour(contour, output, epsilon, curve_threshold, angle_threshold, orientation);
					if (output.contour.size() <= 3)
						bContainCurve = false;
				}
				if (bContainCurve)
				{
					simplified_hole.resize(output.contour.size());
					for (int k = 0; k < output.contour.size(); k++) {
						simplified_hole[k] = output.contour[k];
					}
				}
				else{
					//cv::approxPolyDP(cv::Mat(contour), simplified_hole.points, epsilon, true);
					util::approxPolyDP(cv::Mat(contour), simplified_hole.points, epsilon, true);

				}
				if (simplified_hole.size() >= 3 && util::withinPolygon(simplified_hole, polygon.contour))
					polygon.holes.push_back(simplified_hole);
			}


			polygon.mat = polygon.contour.mat;
			std::vector<std::vector<cv::Point2f>> contours;
			contours = util::tessellate(polygon.contour, polygon.holes);
			for (int i = 0; i < contours.size(); i++) {
				util::clockwise(contours[i]);

				for (int j = 1; j < contours[i].size() - 1; j++) {
					boost::shared_ptr<util::PrimitiveTriangle> pol = boost::shared_ptr<util::PrimitiveTriangle>(new util::PrimitiveTriangle(polygon.mat));
					pol->points.push_back(cv::Point2f(contours[i][0].x, contours[i][0].y));
					pol->points.push_back(cv::Point2f(contours[i][j].x, contours[i][j].y));
					pol->points.push_back(cv::Point2f(contours[i][j + 1].x, contours[i][j + 1].y));
					polygon.primitive_shapes.push_back(pol);
				}
			}
		}
	}

	/**
	* Helper function to extract contours from the input image.
	* The image has to be of type CV_8U, and has values either 0 or 255.
	* Note that the input image is not modified by this function.
	*/
	std::vector<util::Polygon> CurveRightAngleSimplification::findContours(const cv::Mat_<uchar>& img) {
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

	/**
	* @return			false:not curve true: curve
	*/
	bool CurveRightAngleSimplification::approxContour(std::vector<cv::Point2f>& input, util::Polygon &output, float epsilon, float curve_threshold, float angle_threshold, float orientation){
		bool bContainCurve = 0;
		std::vector<cv::Point2f> clean_contour;
		std::vector<int> contour_points_type;
		std::vector<cv::Point3f> contour_points_circle;
		clean_contour.resize(input.size());
		contour_points_type.resize(input.size());
		contour_points_circle.resize(input.size());
		for (int i = 0; i < input.size(); i++){
			clean_contour[i] = cv::Point2f(0.0f, 0.0f);
			contour_points_type[i] = 0;
			contour_points_circle[i] = cv::Point3f(0, 0, 0);
		}
		std::vector<cv::Point2f> clean_contour_tmp;
		cv::Point3d init_points(20, 20, 10);
		std::vector<cv::Point2d> points;
		// start with 15% of all pixels on the contour
		int dis = 0;
		cv::Rect bbox = cv::boundingRect(cv::Mat(input));
		int next = 1;
		double threshold = curve_threshold;
		double percentage = 0.15;
		bool bValid = false;
		int next_p = 1;
		cv::Point3f last_result;
		//std::cout << "input.size() size is " << input.size() << std::endl;
		for (int i = 0; i < input.size();){
			// initialize variables
			points.clear();
			init_points = cv::Point3d(20, 20, 10);
			dis = (int)(percentage * input.size());
			if (dis < 100 && input.size() > 100)
				dis = 100;
			if (dis > 150)
				dis = 150;
			bValid = false;
			next_p = 1;
			do{
				points.clear();
				for (int j = i; j < i + dis; j++){
					double x = input[j % input.size()].x;
					double y = input[j % input.size()].y;
					points.push_back(cv::Point2d(x, y));
				}
				cv::Point3d result = optimizeByBFGS(points, init_points);
				cv::Point2d center(result.x, result.y);
				if (valid_curve(threshold, points, center, result.z, bbox) && points.size() <= input.size()){
					bValid = true;
					next_p = dis;
					dis += 5;
				}
				else{
					if (next_p == 1){
						clean_contour[(i) % input.size()] = input[(i) % input.size()];
						contour_points_type[(i) % input.size()] = 1;
					}
					else{
						// add one more check for small angle 
						float check_angle = 0.0f;
						{
							cv::Point2f center(last_result.x, last_result.y);
							cv::Point2f start = (clean_contour_tmp[0] - center);
							cv::Point2f mid = (clean_contour_tmp[clean_contour_tmp.size() / 2] - center);
							cv::Point2f end = (clean_contour_tmp[clean_contour_tmp.size() - 1] - center);
							cv::Point2f x_axis(1.0, 0);
							check_angle = compute_interval(start, mid, end);
						}
						if (abs(check_angle) >= 30){
							for (int k = 0; k < clean_contour_tmp.size(); k++){
								clean_contour[(i + k) % input.size()] = clean_contour_tmp[k];
								contour_points_type[(i + k) % input.size()] = 2;
								contour_points_circle[(i + k) % input.size()] = last_result;
							}
						}
						else{
							next_p = 1;
							clean_contour[(i) % input.size()] = input[(i) % input.size()];
							contour_points_type[(i) % input.size()] = 1;
						}
					}
					i += next_p;
					bValid = false;
					break;
				}
				// store last clean points
				clean_contour_tmp.clear();
				cv::Point2f start = (points[0] - center);
				cv::Point2f mid = (points[points.size() / 2] - center);
				cv::Point2f end = (points[points.size() - 1] - center);
				cv::Point2f x_axis(1.0, 0);
				float angle_start = compute_angle(x_axis, start);
				float angle_start_end = compute_interval(start, mid, end);
				float interval_angle = angle_start_end / (points.size() - 1);
				for (int k = 0; k < points.size(); k++){
					double x = abs(result.z) * cos(CV_PI * (angle_start + interval_angle * k) / 180) + center.x;
					double y = abs(result.z) * sin(CV_PI * (angle_start + interval_angle * k) / 180) + center.y;
					clean_contour_tmp.push_back(cv::Point2f(x, y));
				}
				last_result = cv::Point3f(result.x, result.y, result.z);

			} while (bValid);

		}
		// simplify points whose type is 1
		std::vector<cv::Point2f> simplified_tmp;
		std::vector<cv::Point2f> simplified_poly;
		std::vector<cv::Point2f> simplified_curve_tmp;
		std::vector<cv::Point2f> final_contour;
		std::vector<int> type_final_contour;
		std::vector<cv::Point3f> final_contour_curve;
		int start_init_tmp = -1;
		bool bConcaveCurve = false;
		for (int i = 0; i < contour_points_type.size(); i++){
			if (contour_points_type[i] == 1){
				start_init_tmp = i;
				break;
			}
		}
		//find the start position of one curve
		int start_init = -1;
		for (int i = start_init_tmp; i < start_init_tmp + contour_points_type.size(); i++){
			if (contour_points_type[i % contour_points_type.size()] == 2){
				start_init = i % contour_points_type.size();
				break;
			}
		}
		if (start_init == -1)
			return false;

		for (int i = start_init; i < start_init + contour_points_type.size();){
			simplified_curve_tmp.clear();
			bool bCurve = false;
			bool bHead = true;
			cv::Point2f curve_center_last;
			cv::Point2f curve_center_current;
			while (contour_points_type[i % contour_points_type.size()] == 2 && i < start_init + contour_points_type.size()){
				bCurve = true;
				if (bHead){
					curve_center_last = cv::Point2f(contour_points_circle[i % contour_points_type.size()].x, contour_points_circle[i % contour_points_type.size()].y);
					bHead = false;
					simplified_curve_tmp.push_back(clean_contour[i % contour_points_type.size()]);
					i++;
				}
				else{
					curve_center_current = cv::Point2f(contour_points_circle[i % contour_points_type.size()].x, contour_points_circle[i % contour_points_type.size()].y);
					float dis = cv::norm(curve_center_last - curve_center_current);
					//std::cout << "dis is " << dis << std::endl;
					if (dis < 1.0f)
					{
						simplified_curve_tmp.push_back(clean_contour[i % contour_points_type.size()]);
						i++;
					}
					else{
						break;
					}
				}
			}
			// simplify the curve using less points
			if (bCurve){
				bContainCurve = true;
				cv::Point2f center = find_center(simplified_curve_tmp[0], simplified_curve_tmp[simplified_curve_tmp.size() / 2], simplified_curve_tmp[simplified_curve_tmp.size() - 1]);
				cv::Point2f start = simplified_curve_tmp[0] - center;
				cv::Point2f mid = simplified_curve_tmp[simplified_curve_tmp.size() / 2] - center;
				cv::Point2f end = simplified_curve_tmp[simplified_curve_tmp.size() - 1] - center;
				cv::Point2f x_axis(1.0, 0);
				float angle_start = compute_angle(x_axis, start);
				float angle_start_end = compute_interval(start, mid, end);
				int degrees = 8;
				int interval_points = abs(angle_start_end / degrees);
				float interval_degrees = angle_start_end / interval_points;
				float radius = cv::norm(simplified_curve_tmp[0] - center);

				if ((abs(angle_start_end) >= 60 && radius > 30) || (abs(angle_start_end) >= 180 && radius > 20)){
					for (int k = 0; k < interval_points; k++){
						float x = radius * cos(CV_PI * (angle_start + interval_degrees * k) / 180) + center.x;
						float y = radius * sin(CV_PI * (angle_start + interval_degrees * k) / 180) + center.y;
						final_contour.push_back(cv::Point2f(x, y));
						final_contour_curve.push_back(cv::Point3f(center.x, center.y, radius));
						type_final_contour.push_back(2);
					}
					final_contour.push_back(simplified_curve_tmp[simplified_curve_tmp.size() - 1]);
					final_contour_curve.push_back(cv::Point3f(center.x, center.y, radius));
					type_final_contour.push_back(2);
				}
				else{
					simplified_poly.clear();
					cv::approxPolyDP(cv::Mat(simplified_curve_tmp), simplified_poly, epsilon, false);
					for (int k = 0; k < simplified_poly.size(); k++){
						final_contour.push_back(simplified_poly[k]);
						final_contour_curve.push_back(cv::Point3f(0, 0, 0));
						type_final_contour.push_back(1);
					}
				}
			}

			simplified_tmp.clear();
			simplified_poly.clear();
			bool bNon_Curve = false;
			while (contour_points_type[i % contour_points_type.size()] == 1 && i < start_init + contour_points_type.size()){
				bNon_Curve = true;
				simplified_tmp.push_back(clean_contour[i % contour_points_type.size()]);
				i++;
			}
			if (bNon_Curve){
				cv::approxPolyDP(cv::Mat(simplified_tmp), simplified_poly, epsilon, false);
				for (int k = 0; k < simplified_poly.size(); k++){
					final_contour.push_back(simplified_poly[k]);
					final_contour_curve.push_back(cv::Point3f(0, 0, 0));
					type_final_contour.push_back(1);
				}
			}

		}
		if (bContainCurve){
			//get maximal direction angle
			float angle = orientation * 180 / CV_PI;
			// transform
			cv::Mat_<float> M;
			final_contour = transform_angle(final_contour, M, angle);

			// generate output polygon and decomposePolygon here
			output.contour.resize(final_contour.size());
			bConcaveCurve = false;
			int start_index = 0;
			bool bfind_start = false;
			int regular_points = 0;
			// find the index of starting point
			for (int i = 0; i < final_contour.size(); i++){
				output.contour[i] = final_contour[i];
				if (type_final_contour[i] == 1){
					if (!bfind_start)
						start_index = i;
					bfind_start = true;
					regular_points ++;
				}
			}
			// check whether the polygon is simple
			if (!util::isSimple(output.contour))
				return false;

			// determine curves

			bool bCurve = false;
			std::vector<cv::Point2f> one_curve;
			std::vector<cv::Point2f> one_curve_attrs;
			cv::Point2f center;
			std::vector<std::vector<cv::Point2f>> output_curves;
			std::vector<std::vector<cv::Point2f>> output_curves_attrs;
			std::vector<std::vector<cv::Point2f>> output_curves_rectify;
			std::vector<std::vector<cv::Point2f>> output_curves_attrs_rectify;
			float radius;
			for (int i = start_index; i < start_index + final_contour.size();){
				int tmp = i % final_contour.size();

				if (type_final_contour[tmp] == 1){
					bCurve = false;
					i++;
				}
				one_curve.clear();
				one_curve_attrs.clear();
				int curve_start_index = 0;
				int curve_end_index = 0;
				while (type_final_contour[tmp] == 2 && i < start_index + final_contour.size()){
					if (!bCurve){
						center = cv::Point2f(final_contour_curve[tmp].x, final_contour_curve[tmp].y);
						radius = final_contour_curve[tmp].z;
						curve_start_index = tmp;
					}
					one_curve.push_back(final_contour[tmp]);
					i++;
					tmp = i % final_contour.size();
					curve_end_index = tmp;
					bCurve = true;
				}
				if (bCurve){
					cv::Point2f start = one_curve[0];
					cv::Point2f mid = one_curve[one_curve.size() / 2];
					cv::Point2f end = one_curve[one_curve.size() - 1];
					cv::Point2f x_axis(1.0, 0);
					// check whether it's a concave curve 
					std::vector<cv::Point2f> end_points;
					end_points.push_back(start);
					end_points.push_back(end);
					if (concaveCurve(end_points, output) && one_curve.size() < final_contour.size()){
						bConcaveCurve = true;
						if (curve_start_index >= curve_end_index){
							for (int j = curve_start_index; j < curve_end_index + final_contour.size(); j++)
								type_final_contour[j % final_contour.size()] = 3;
						}
						else{
							for (int j = curve_start_index; j < curve_end_index; j++)
								type_final_contour[j] = 3;
						}
						continue;
					}
					one_curve_attrs.push_back(start);
					one_curve_attrs.push_back(mid);
					one_curve_attrs.push_back(end);
					one_curve_attrs.push_back(center);
					output_curves.push_back(one_curve);
					output_curves_attrs.push_back(one_curve_attrs);
				}
			}

			// determine regular points
			std::vector<cv::Point2f> output_regular;
			std::vector<cv::Point2f> concave_curve_tmp;
			std::vector<cv::Point2f> concave_curve_simp_tmp;
			for (int i = start_index; i < start_index + final_contour.size();){
				if (type_final_contour[i % final_contour.size()] == 1){
					output_regular.push_back(final_contour[i % final_contour.size()]);
					i++;
				}
				//
				bool curveHead = false;
				while (type_final_contour[i % final_contour.size()] == 2 && i < start_index + final_contour.size()){
					if (!curveHead){
						curveHead = true;
						output_regular.push_back(final_contour[i % final_contour.size()]);
						//std::cout << "start " << final_contour[i % final_contour.size()]<<std::endl;
					}
					i++;
					if (type_final_contour[i % final_contour.size()] == 1){
						output_regular.push_back(final_contour[(i - 1) % final_contour.size()]);
						//std::cout << "end " << final_contour[(i - 1) % final_contour.size()] << std::endl;
					}

				}
				//
				concave_curve_tmp.clear();
				concave_curve_simp_tmp.clear();
				bool concaveCurve = false;
				while (type_final_contour[i % final_contour.size()] == 3 && i < start_index + final_contour.size()){
					concaveCurve = true;
					concave_curve_tmp.push_back(final_contour[i % final_contour.size()]);
					i++;
				}
				if (concaveCurve){
					//cv::approxPolyDP(cv::Mat(concave_curve_tmp), concave_curve_simp_tmp, epsilon, false);
					//for (int m = 0; m < concave_curve_simp_tmp.size(); m++)
					//	output_regular.push_back(concave_curve_simp_tmp[m]);
					for (int m = 0; m < concave_curve_tmp.size(); m++)
						output_regular.push_back(concave_curve_tmp[m]);
				}


			}

			// The transfomration matrix should be same for the external contour and the internal holes
			// because for OpenCV simplification, transformation is just to trasnform from OpenCV image coordinates
			// to the world coordinate system.
			output.mat = output.contour.mat;
			// find primitives
			output.primitive_shapes.clear();
			output.contour.points = transform(output.contour.points, M.inv());
			//std::cout << "bConcaveCurve is " << bConcaveCurve << std::endl;
			//std::cout << "regular_points is " << regular_points << std::endl;
			//std::cout << "output_curves is " << output_curves.size() << std::endl;
			if (bConcaveCurve || regular_points < 4){
				// a. curve primitives
				for (int i = 0; i < output_curves.size(); i++){
					output_curves_attrs[i] = transform(output_curves_attrs[i], M.inv());
					center = find_center(output_curves_attrs[i][0], output_curves_attrs[i][1], output_curves_attrs[i][2]);
					cv::Point2f start = output_curves_attrs[i][0] - center;
					cv::Point2f mid = output_curves_attrs[i][1] - center;
					cv::Point2f end = output_curves_attrs[i][2] - center;
					cv::Point2f x_axis(1.0, 0);
					float theta_start = compute_angle(x_axis, start);
					float theta_start_end = compute_interval(start, mid, end);
					float theta_end = theta_start + theta_start_end;
					radius = cv::norm(start);
					//if (abs(theta_start_end) <= 5){
					//	std::cout << "theta_start_end " << theta_start_end << std::endl;
					//}
					//if (std::isnan(theta_start) || std::isnan(theta_end)) {
					//	std::cout << "NaN " << std::endl;
					//}
					boost::shared_ptr<util::PrimitiveCurve> pol = boost::shared_ptr<util::PrimitiveCurve>(new util::PrimitiveCurve(output.mat, theta_start, theta_end, center, radius));
					output.primitive_shapes.push_back(pol);
				}

				// b. triangle primitives
				regular_points = 0;
				for (int i = 0; i < final_contour.size(); i++){
					if (type_final_contour[i] == 1 || type_final_contour[i] == 3){
						regular_points++;
					}
				}
				if (regular_points == 0)
					return bContainCurve;
				util::Polygon new_polygon;
				//new_polygon.contour.resize(regular_points);
				//int index = 0;
				//for (int i = 0; i < final_contour.size(); i++){
				//	if (type_final_contour[i] == 1 || type_final_contour[i] == 3){
				//		new_polygon.contour[index++] = final_contour[i];
				//	}
				//}
				new_polygon.contour.resize(output_regular.size());
				int index = 0;
				for (int i = 0; i < output_regular.size(); i++){
					new_polygon.contour[i] = output_regular[i];

				}

				new_polygon.contour.points = transform(new_polygon.contour.points, M.inv());
				std::vector<std::vector<cv::Point2f>> contours;
				contours = util::tessellate(new_polygon.contour.points);

				for (int i = 0; i < contours.size(); i++) {
					util::clockwise(contours[i]);

					for (int j = 1; j < contours[i].size() - 1; j++) {
						boost::shared_ptr<util::PrimitiveTriangle> pol = boost::shared_ptr<util::PrimitiveTriangle>(new util::PrimitiveTriangle(output.mat));
						pol->points.push_back(cv::Point2f(contours[i][0].x, contours[i][0].y));
						pol->points.push_back(cv::Point2f(contours[i][j].x, contours[i][j].y));
						pol->points.push_back(cv::Point2f(contours[i][j + 1].x, contours[i][j + 1].y));
						output.primitive_shapes.push_back(pol);
					}
				}
				//std::cout << "num_curves is " << num_curves << std::endl;
				return bContainCurve;
			}
			else{

				regular_points = 0;
				for (int i = 0; i < final_contour.size(); i++){
					if (type_final_contour[i] == 1 || type_final_contour[i] == 3){
						regular_points++;
					}
				}
				//if (regular_points == 0)
				//	return bContainCurve;
				//std::cout << "regular_points is " << regular_points << std::endl;
				//std::cout << "output_regular is " << output_regular.size() << std::endl;
				output.contour.clear();
				//
				std::vector<cv::Point2f> output_regular_tmp;
				output_regular_tmp.resize(output_regular.size());
				for (int i = 0; i < output_regular.size(); i++){
					output_regular_tmp[i] = output_regular[i];
				}
				output_regular.clear();
				//cv::approxPolyDP(cv::Mat(output_regular_tmp), output_regular, epsilon, true);
				util::approxPolyDP(cv::Mat(output_regular_tmp), output_regular, epsilon, true);
				std::vector<cv::Point2f> results_tmp;
				std::vector<cv::Point2f> new_output_contour;
				results_tmp = contour_rectify(output_regular, angle_threshold, epsilon);
				std::vector<cv::Point2f> results = del_redundant_points(results_tmp);
				if (!util::isSimple(results)){
					results = results_tmp;
				}		
				//std::cout << "results is " << results.size() << std::endl;
				float snap_threshold = 7;
				//std::cout << "output_curves is " << output_curves.size() << std::endl;
				//std::cout << "output_curves 0 is " << output_curves[0].size() << std::endl;
				new_output_contour = rectify_curves(results, output_curves, output_curves_attrs, output_curves_rectify, output_curves_attrs_rectify, snap_threshold);

				// a. curve primitives
				for (int i = 0; i < output_curves_rectify.size(); i++){
					output_curves_attrs_rectify[i] = transform(output_curves_attrs_rectify[i], M.inv());
					cv::Point2f start = output_curves_attrs_rectify[i][0] - output_curves_attrs_rectify[i][3];
					cv::Point2f mid = output_curves_attrs_rectify[i][1] - output_curves_attrs_rectify[i][3];
					cv::Point2f end = output_curves_attrs_rectify[i][2] - output_curves_attrs_rectify[i][3];
					cv::Point2f x_axis(1.0, 0);
					float theta_start = compute_angle(x_axis, start);
					float theta_start_end = compute_interval(start, mid, end);
					float theta_end = theta_start + theta_start_end;
					center = output_curves_attrs_rectify[i][3];
					radius = cv::norm(start);
					//if (abs(theta_start_end) <= 5){
					//	std::cout << "theta_start_end " << theta_start_end << std::endl;
					//}
					//if (std::isnan(theta_start) || std::isnan(theta_end)) {
					//	std::cout << "NaN " << std::endl;
					//}
					boost::shared_ptr<util::PrimitiveCurve> pol = boost::shared_ptr<util::PrimitiveCurve>(new util::PrimitiveCurve(output.mat, theta_start, theta_end, center, radius));
					output.primitive_shapes.push_back(pol);
				}

				// b. triangle primitives
				util::Polygon new_polygon;
				new_polygon.contour = results;
				new_polygon.contour.points = transform(new_polygon.contour.points, M.inv());
				std::vector<std::vector<cv::Point2f>> contours;
				contours = util::tessellate(new_polygon.contour);

				for (int i = 0; i < contours.size(); i++) {
					util::clockwise(contours[i]);

					for (int j = 1; j < contours[i].size() - 1; j++) {
						boost::shared_ptr<util::PrimitiveTriangle> pol = boost::shared_ptr<util::PrimitiveTriangle>(new util::PrimitiveTriangle(output.mat));
						pol->points.push_back(cv::Point2f(contours[i][0].x, contours[i][0].y));
						pol->points.push_back(cv::Point2f(contours[i][j].x, contours[i][j].y));
						pol->points.push_back(cv::Point2f(contours[i][j + 1].x, contours[i][j + 1].y));
						output.primitive_shapes.push_back(pol);
					}
				}

				//update output polygon
				new_output_contour = transform(new_output_contour, M.inv());
				output.contour = new_output_contour;
				//std::cout << "num_curves is " << num_curves << std::endl;
				return bContainCurve;

			}
		}
		else
			return bContainCurve;
	}

	bool CurveRightAngleSimplification::concaveCurve(std::vector<cv::Point2f>& end_points, util::Polygon &polygon){
		cv::Point2f start_point = end_points[0];
		cv::Point2f end_point = end_points[1];

		// sample some points
		int outside = 0;
		for (int i = 1; i < 11; i++){
			float x = start_point.x + i * (end_point.x - start_point.x) / 11;
			float y = start_point.y + i * (end_point.y - start_point.y) / 11;
			cv::Point2f point(x, y);
			if (!withinPolygon(point, polygon))
				outside++;
		}
		//std::cout << "outside is " << outside << std::endl;
		if (outside > 5)
			return true;
		else
			return false;

	}

	cv::Point3f CurveRightAngleSimplification::optimizeByBFGS(const std::vector<cv::Point2d>& points, cv::Point3d init_points) {
		cv::Point3d ans;

		try {
			column_vector starting_point(3);
			starting_point(0) = init_points.x;
			starting_point(1) = init_points.y;
			starting_point(2) = init_points.z;

			BFGSSolver solver(points);
			BFGSSolverDeriv solverD(points);
			find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), solver, solverD, starting_point, -1);

			ans.x = starting_point(0);
			ans.y = starting_point(1);
			ans.z = starting_point(2);
		}
		catch (std::exception& ex) {
			std::cout << "BFGS optimization failure" << std::endl;
		}

		return ans;
	}

	float CurveRightAngleSimplification::compute_angle(cv::Point2d a, cv::Point2d b){
		double norm_a = cv::norm(a);
		double norm_b = cv::norm(b);
		a.x = a.x / norm_a;
		a.y = a.y / norm_a;
		b.x = b.x / norm_b;
		b.y = b.y / norm_b;
		/*double dot = a.x*b.x + a.y*b.y;
		double ans = acos(dot);
		return ans / CV_PI * 180;*/
		double dot = a.x*b.x + a.y*b.y;
		double det = a.x*b.y - a.y*b.x;
		double angle = atan2(det, dot);
		return angle / CV_PI * 180;
		//return angle;
	}

	float CurveRightAngleSimplification::compute_abs_angle(cv::Point2d a, cv::Point2d b){
		double norm_a = cv::norm(a);
		double norm_b = cv::norm(b);
		a.x = a.x / norm_a;
		a.y = a.y / norm_a;
		b.x = b.x / norm_b;
		b.y = b.y / norm_b;
		double dot = a.x*b.x + a.y*b.y;
		double ans = acos(dot);
		return ans / CV_PI * 180;
	}

	float CurveRightAngleSimplification::compute_interval(cv::Point2d start, cv::Point2d mid, cv::Point2d end){
		cv::Point2f x_axis(1.0, 0);
		float angle_start = compute_angle(x_axis, start);
		float angle_mid = compute_angle(x_axis, mid);
		float angle_end = compute_angle(x_axis, end);
		//std::cout << "start is " << start << std::endl;
		//std::cout << "mid is " << mid << std::endl;
		//std::cout << "end is " << end << std::endl;
		// checking increse
		float start_tmp = angle_start;
		float mid_tmp = angle_mid;
		float end_tmp = angle_end;
		if (start_tmp > mid_tmp)
			mid_tmp += 360;
		if (mid_tmp > end_tmp)
			end_tmp += 360;
		if (start_tmp <= mid_tmp && mid_tmp <= end_tmp && (end_tmp - start_tmp) <= 360)
			return end_tmp - start_tmp;

		// checking decrease
		start_tmp = angle_start;
		mid_tmp = angle_mid;
		end_tmp = angle_end;
		if (start_tmp < mid_tmp)
			mid_tmp -= 360;
		if (mid_tmp < end_tmp)
			end_tmp -= 360;
		if (start_tmp >= mid_tmp && mid_tmp >= end_tmp && (end_tmp - start_tmp) >= -360)
			return end_tmp - start_tmp;
		return end_tmp - start_tmp;
	}

	bool CurveRightAngleSimplification::valid_curve(double threshold, const std::vector<cv::Point2d>& points, cv::Point2d center, double radius, cv::Rect bbox){
		bool bValid = true;
		// compute max_error
		double max_error = 0;
		std::vector<double> errors;
		errors.resize(points.size());
		for (int i = 0; i < points.size(); i++){
			double tmp = abs((cv::norm(center - points.at(i)) - abs(radius)));
			//tmp= glm::pow((glm::length2(center - points.at(i)) - radius*radius), 2);
			errors[i] = tmp;
			if (tmp > max_error)
				max_error = tmp;
		}
		std::sort(errors.begin(), errors.end());
		float ratio = 0.90;
		max_error = errors[errors.size() * ratio];
		// compute angle_v1
		float angle_v1 = compute_interval(points[0] - center, points[points.size() / 2] - center, points[points.size() - 1] - center);
		//double angle_v1 = abs(compute_angle(points[0] - center, points[points.size() - 1] - center));
		// compute angle_v2
		double angle_v2 = abs(compute_abs_angle(points[points.size() / 2] - points[points.size() / 4], points[points.size() * 3 / 4] - points[points.size() / 2]));
		// compute angle_v3
		double angle_v3 = abs(compute_abs_angle(points[points.size() / 4] - points[0], points[points.size() / 3] - points[points.size() / 4]));
		// compute angle_v4
		double angle_v4 = abs(compute_abs_angle(points[points.size() * 3 / 4] - points[points.size() * 2 / 3], points[points.size() - 1] - points[points.size() * 3 / 4]));
		//std::cout << "The max_error is " << max_error << std::endl;
		//std::cout << "The angle_v1 is " << angle_v1 << std::endl;
		//std::cout << "The angle_v2 is " << angle_v2 << std::endl;
		double diaginal = sqrt(pow(bbox.width, 2) + pow(bbox.height, 2));
		//std::cout << "The diaginal length is " << diaginal << std::endl;
		// threshold and error comparison
		if (max_error > threshold)
			return false;
		// angle v1 comparison
		if (abs(angle_v1) < 10)
			return false;
		// angle v2 comparison
		/*
		if (angle_v2 < 5)
			return false;
		if (angle_v3 < 5)
			return false;
		if (angle_v4 < 5)
			return false;
			*/
		// center comparison
		{

		}
		// radius comparison
		if (radius > 0.5 * diaginal)
			return false;
		/*if (radius < )*/
		return true;
	}

	float CurveRightAngleSimplification::axis_align(const cv::Mat_<uchar>& src){
		cv::Mat dst;
		cv::Canny(src, src, 50, 150, 3);
		dst = src;
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(dst, lines, 1, CV_PI / 180, 20, 1, 5);
		std::vector<std::pair <float, float>> edges;
		edges.resize(lines.size());
		for (int i = 0; i < lines.size(); i++)
		{
			cv::Vec4i l = lines[i];
			cv::Point tail(l[0], l[1]);
			cv::Point head(l[2], l[3]);
			cv::Point2d vec1(tail.x - head.x, tail.y - head.y);
			cv::Point2d vec2(1, 0);
			cv::Point2d vec3(0, 1);
			if (l[1] > l[3])
				vec1 = cv::Point2d(tail.x - head.x, tail.y - head.y);
			else if (l[1] = l[3])
			{
				if (l[0] >= l[2])
					vec1 = cv::Point2d(tail.x - head.x, tail.y - head.y);
				else
					vec1 = -cv::Point2d(tail.x - head.x, tail.y - head.y);
			}
			else
				vec1 = -cv::Point2d(tail.x - head.x, tail.y - head.y);
			float angle = compute_abs_angle(vec1, vec2);
			edges[i] = std::make_pair(angle, cv::norm(tail - head));
		}

		std::unordered_map<int, float> mymap;
		boost::math::normal s(0, 20);
		for (int i = 0; i < edges.size(); i++){
			for (int j = 0; j < 180; j++){
				int index = (int)(j - edges[i].first);
				float vote = edges[i].second * pdf(s, index);
				std::pair<int, double> node(j, vote);
				mymap.insert(node);
			}
		}
		float result = 0;
		float tmp = 0;
		for (auto it = mymap.begin(); it != mymap.end(); ++it){
			if (it->second > tmp){
				result = it->first;
				tmp = it->second;
			}
		}
		return result;
	}

	std::vector<cv::Point2f> CurveRightAngleSimplification::transform_angle(std::vector<cv::Point2f> contour, cv::Mat_<float> &M, float angle){
		// computer the center of the contour
		cv::Point2f center(0, 0);
		for (int i = 0; i < contour.size(); i++){
			center.x += contour[i].x;
			center.y += contour[i].y;
		}
		center.x = center.x / contour.size();
		center.y = center.y / contour.size();
		double theta = angle / 180 * CV_PI;

		// create a transformation matrix
		cv::Mat_<float> M_translate = (cv::Mat_<float>(3, 3) << 1, 0, center.x, 0, 1, center.y, 0, 0, 1);
		cv::Mat_<float> M_rotate = (cv::Mat_<float>(3, 3) << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1);
		cv::Mat_<float> M_translate_inv = (cv::Mat_<float>(3, 3) << 1, 0, -center.x, 0, 1, -center.y, 0, 0, 1);
		M = M_translate * M_rotate * M_translate_inv;
		// create inverse transformation matrix
		cv::Mat_<float> invM = M.inv();

		// transform the polygon
		std::vector<cv::Point2f> aa_contour(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << contour[i].x, contour[i].y, 1);
			cv::Mat_<float> p2 = M * p;
			aa_contour[i] = cv::Point2f(p2(0, 0), p2(1, 0));
		}
		return aa_contour;
	}

	std::vector<cv::Point2f> CurveRightAngleSimplification::transform(std::vector<cv::Point2f> contour, cv::Mat_<float> M){
		// transform the polygon
		std::vector<cv::Point2f> aa_contour(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << contour[i].x, contour[i].y, 1);
			cv::Mat_<float> p2 = M * p;
			aa_contour[i] = cv::Point2f(p2(0, 0), p2(1, 0));
		}
		return aa_contour;
	}

	std::vector<cv::Point2f> CurveRightAngleSimplification::rectify_curves(std::vector<cv::Point2f> contour, std::vector<std::vector<cv::Point2f>> curves, std::vector<std::vector<cv::Point2f>> curves_attrs, std::vector<std::vector<cv::Point2f>>& curves_rectify, std::vector<std::vector<cv::Point2f>>& curves_attrs_rectify, float snap_threshold){
		typedef boost::geometry::model::d2::point_xy<float> point_t;
		typedef boost::geometry::model::segment<point_t> Segment;

		curves_rectify.resize(curves.size());
		curves_attrs_rectify.resize(curves.size());
		std::vector<cv::Point2f> curve_tmp;
		std::vector<cv::Point2f> curve_attrs_tmp;
		std::vector<cv::Point2f> result_contour;
		std::vector<int> insert_pos;
		insert_pos.resize(curves.size());
		std::vector<std::pair<bool, bool>> insert_pos_new;
		insert_pos_new.resize(curves.size());
		for (int i = 0; i < curves.size(); i++){
			curve_tmp.clear();
			curve_attrs_tmp.clear();
			cv::Point2f center, start_point, mid_point, end_point;
			start_point = cv::Point2f(curves_attrs[i][0].x, curves_attrs[i][0].y);
			mid_point = cv::Point2f(curves_attrs[i][1].x, curves_attrs[i][1].y);
			end_point = cv::Point2f(curves_attrs[i][2].x, curves_attrs[i][2].y);
			center = cv::Point2f(curves_attrs[i][3].x, curves_attrs[i][3].y);
			//std::cout << "start_point is " << start_point << std::endl;
			//std::cout << "end_point is " << end_point << std::endl;
			//std::cout << "mid_point is " << mid_point << std::endl;
			double start_dis = 0.0f, end_dis = 0.0;
			cv::Point2f tmp = (start_point + end_point) * 0.5;
			cv::Point2f dir = (tmp - mid_point) / cv::norm(tmp - mid_point);
			cv::Point2f new_point = tmp + 40 * dir;
			Segment CD(point_t(mid_point.x, mid_point.y), point_t(new_point.x, new_point.y));

			int start_index = -1, end_index = -1;
			for (int j = 0; j < contour.size(); j++){
				int tail = j;
				int head = (j + 1) % contour.size();
				Segment AB(point_t(contour[tail].x, contour[tail].y), point_t(contour[head].x, contour[head].y));
				bool result = boost::geometry::intersects(AB, CD);
				if (result){
					start_index = tail;
					end_index = head;
					break;
				}
			}
			if (start_index != -1 && end_index != -1){
				// decide swap start point or not
				float dis_1 = cv::norm(start_point - contour[start_index]);
				float dis_2 = cv::norm(end_point - contour[start_index]);
				insert_pos[i] = start_index;
				if (dis_1 > dis_2){
					cv::Point2f tmp = start_point;
					start_point = end_point;
					end_point = tmp;
				}
				start_dis = cv::norm(start_point - contour[start_index]);
				end_dis = cv::norm(end_point - contour[end_index]);
				if (start_dis <= snap_threshold && start_index != end_index){
					start_point = contour[start_index];
					insert_pos_new[i].first = false;
				}
				else{
					insert_pos_new[i].first = true;
					// find intersection
					//cv::Point2f dir = (start_point - mid_point) / cv::norm(start_point - mid_point);
					//cv::Point2f new_point = start_point + 10 * dir;
					//Segment seg_tmp1(point_t(new_point.x, new_point.y), point_t(start_point.x, start_point.y));
					//Segment seg_tmp2(point_t(contour[start_index].x, contour[start_index].y), point_t(contour[end_index].x, contour[end_index].y));
					//std::vector<point_t> output;
					//boost::geometry::intersection(seg_tmp1, seg_tmp2, output);
					//std::cout << "output size  is  " << output.size() << std::endl;
					//std::cout << "start_point  is  " << start_point << std::endl;
					//std::cout << "new_point  is  " << new_point << std::endl;
					//if (output.size() == 1){
					//	start_point = cv::Point2f(output[0].x(), output[0].y());
					//}
					// sample some points on the line
					std::vector<cv::Point2f> smaple_points;
					int sample_num = 50;
					float tmp_dis = 1000;
					int tmp_index = 0;
					float interval_x = (contour[end_index].x - contour[start_index].x) / sample_num;
					float interval_y = (contour[end_index].y - contour[start_index].y) / sample_num;
					for (int k = 1; k < sample_num; k++){
						cv::Point2f new_point(contour[start_index].x + interval_x * k, contour[start_index].y + interval_y * k);
						smaple_points.push_back(new_point);
						float new_dis = cv::norm(new_point - start_point);
						if (tmp_dis > new_dis){
							tmp_index = k;
							tmp_dis = new_dis;
						}
					}
					start_point = smaple_points[tmp_index - 1];


				}
				if (end_dis <= snap_threshold && start_index != end_index){
					end_point = contour[end_index];
					insert_pos_new[i].second = false;
				}
				else{
					insert_pos_new[i].second = true;
					std::vector<cv::Point2f> smaple_points;
					int sample_num = 50;
					float tmp_dis = 1000;
					int tmp_index = 0;
					float interval_x = (contour[end_index].x - contour[start_index].x) / sample_num;
					float interval_y = (contour[end_index].y - contour[start_index].y) / sample_num;
					for (int k = 1; k < sample_num; k++){
						cv::Point2f new_point(contour[start_index].x + interval_x * k, contour[start_index].y + interval_y * k);
						smaple_points.push_back(new_point);
						float new_dis = cv::norm(new_point - end_point);
						if (tmp_dis > new_dis){
							tmp_index = k;
							tmp_dis = new_dis;
						}
					}
					end_point = smaple_points[tmp_index-1];
				}

			}
			else{
				insert_pos[i] = -1;
			}


			// get the new curve
			center = find_center(start_point, mid_point, end_point);
			//if (std::isnan(center.x)) {
			//	std::cout << "center NAN " << std::endl;
			//	std::cout << "contour.size() is " << contour.size() << std::endl;
			//	for (int k = 0; k < contour.size(); k++)
			//		std::cout << "point is " << contour[k] << std::endl;
			//	std::cout << "ori start_point is " << cv::Point2f(curves_attrs[i][0].x, curves_attrs[i][0].y) << std::endl;
			//	std::cout << "ori mid_point is " << cv::Point2f(curves_attrs[i][1].x, curves_attrs[i][1].y) << std::endl;
			//	std::cout << "ori end_point is " << cv::Point2f(curves_attrs[i][2].x, curves_attrs[i][2].y) << std::endl;
			//	std::cout << "start_point is " << start_point << std::endl;
			//	std::cout << "mid_point is " << mid_point << std::endl;
			//	std::cout << "end_point is " << end_point << std::endl;
			//	std::cout << "start_index is " << start_index << std::endl;
			//}
			cv::Point2f start = start_point - center;
			cv::Point2f mid = mid_point - center;
			cv::Point2f end = end_point - center;
			cv::Point2f x_axis(1.0, 0);
			float angle_start = compute_angle(x_axis, start);
			float angle_start_end = compute_interval(start, mid, end);
			if (abs(angle_start_end) < 30){
				insert_pos[i] = -2;
				continue;
			}
			int degrees = 8;
			int interval_points = abs(angle_start_end / degrees);
			float interval_degrees = angle_start_end / interval_points;
			float radius = cv::norm(start_point - center);
			for (int k = 0; k < interval_points; k++){
				float x = radius * cos(CV_PI * (angle_start + interval_degrees * k) / 180) + center.x;
				float y = radius * sin(CV_PI * (angle_start + interval_degrees * k) / 180) + center.y;
				curve_tmp.push_back(cv::Point2f(x, y));
			}
			curve_attrs_tmp.push_back(start_point);
			curve_attrs_tmp.push_back(mid_point);
			curve_attrs_tmp.push_back(end_point);
			curve_attrs_tmp.push_back(center);
			curve_tmp.push_back(end_point);
			curves_rectify[i] = curve_tmp;
			curves_attrs_rectify[i] = curve_attrs_tmp;
		}
		// update the new contour
		bool bInsert = false;
		int insert_index = 0;
		for (int i = 0; i < contour.size(); i++){
			bInsert = false;
			for (int j = 0; j < insert_pos.size(); j++)
			{
				if (i == insert_pos[j]){
					bInsert = true;
					insert_index = j;
					break;
				}
			}
			if (bInsert){
				if (insert_pos_new[insert_index].first)
					result_contour.push_back(contour[i]);
				for (int k = 0; k < curves_rectify[insert_index].size(); k++){
					if (insert_pos[insert_index] == contour.size() - 1 && k == curves_rectify[insert_index].size() - 1)
						continue;
					else{
						result_contour.push_back(curves_rectify[insert_index][k]);
					}
				}
				if (!insert_pos_new[insert_index].second)
					i++;
			}
			else
				result_contour.push_back(contour[i]);
		}
		// update
		int count = 0;
		for (int i = 0; i < curves.size(); i++){
			if (insert_pos[i] == -2){
				curves_rectify.erase(curves_rectify.begin() + i - count);
				curves_attrs_rectify.erase(curves_attrs_rectify.begin() + i - count);
				count++;
			}
		}

		return result_contour;
	}

	std::vector<cv::Point2f> CurveRightAngleSimplification::del_redundant_points(std::vector<cv::Point2f> contour){
		// find a corner point
		int start_index = 0;
		for (int i = 0; i < contour.size(); i++){
			int tail = i;
			int mid = (i + 1) % contour.size();
			int head = (i + 2) % contour.size();
			float value = contour[tail].x * (contour[mid].y - contour[head].y)
				+ contour[mid].x * (contour[head].y - contour[tail].y)
				+ contour[head].x * (contour[tail].y - contour[mid].y);
			if (value > 0.1){
				start_index = mid;
				break;
			}

		}
		//
		std::vector<cv::Point2f> result;
		for (int i = start_index; i < contour.size() + start_index;){
			float value = 1.0;
			int tail = i % contour.size();
			int mid = (i + 1) % contour.size();
			int head = (i + 2) % contour.size();
			int gap = 0;
			result.push_back(contour[tail]);
			do{
				value = contour[tail].x * (contour[mid].y - contour[head].y)
					+ contour[mid].x * (contour[head].y - contour[tail].y)
					+ contour[head].x * (contour[tail].y - contour[mid].y);
				mid = head;
				head = (head + 1) % contour.size();
				gap++;
			} while (abs(value) <= 0.01 && i + gap < contour.size() + start_index);
			i += gap;
		}
		return result;
	}

	std::vector<cv::Point2f> CurveRightAngleSimplification::contour_rectify(std::vector<cv::Point2f>& contour, float threshold, float epsilon){

		// assume it's axis-aligned contour
		std::vector<cv::Point2f> results;
		// type: 0 regular type, 1 horizontal type, 2 vertical type
		results.resize(contour.size());
		for (int i = 0; i < contour.size(); i++){
			int tail = i;
			int head = (i + 1) % contour.size();
			cv::Point2d vec1(contour[head].x - contour[tail].x, contour[head].y - contour[tail].y);
			cv::Point2d vec2(1, 0);
			cv::Point2d vec3(0, 1);
			float angle = compute_abs_angle(vec1, vec2);
			//std::cout << "tail is " << contour[tail] << std::endl;
			//std::cout << "head is " << contour[head] << std::endl;
			//std::cout << "angle is " << angle << std::endl;

			int type = 0;
			if (angle < threshold || abs(angle - 180) < threshold)
				type = 1;
			else if (abs(angle - 90) < threshold)
				type = 2;
			else
				type = 0;
			if (type == 0){
				//do nothing
			}
			else if (type == 1){
				//contour[head].x = contour[tail].x + dot;
				contour[head].y = contour[tail].y;
			}
			else if (type == 2){
				contour[head].x = contour[tail].x;
				//contour[head].y = contour[tail].y + dot;
			}
			else{
				// do nothing
			}
		}
		util::approxPolyDP(cv::Mat(contour), results, epsilon, true);
		/*for (int i = 0; i < contour.size(); i++)
			results[i] = contour[i];*/
		return results;
	}

	std::vector<cv::Point2f> CurveRightAngleSimplification::contour_rectify_no_curve(std::vector<cv::Point2f>& contour, float threshold, float epsilon){
		// assume it's axis-aligned contour
		std::vector<cv::Point2f> results;
		// type: 0 regular type, 1 horizontal type, 2 vertical type
		results.resize(contour.size());
		for (int i = 0; i < contour.size(); i++){
			int tail = i;
			int head = (i + 1) % contour.size();
			int head_next = (i + 2) % contour.size();
			cv::Point2d vec1(contour[head].x - contour[tail].x, contour[head].y - contour[tail].y);
			cv::Point2d vec2(1, 0);
			cv::Point2d vec3(0, 1);
			float angle = compute_abs_angle(vec1, vec2);
			int type = 0;
			float angle_1 = std::min(angle, (float)abs(angle - 180));
			float angle_2 = abs(angle - 90);
			float angle_3 = std::min((float)abs(angle - 45), (float)abs(angle - 135));

			if (angle_1 < angle_2 && angle_1 < angle_3)
				type = 1;
			else if (angle_2 < angle_1 && angle_2 < angle_3)
				type = 2;
			else
				type = 0;

			if (type == 1){
				//contour[head].x = contour[tail].x + dot;
				contour[head].y = contour[tail].y;
			}
			else if (type == 2){
				contour[head].x = contour[tail].x;
				//contour[head].y = contour[tail].y + dot;
			}
			else{
				// do nothing
			}
		}
		util::approxPolyDP(cv::Mat(contour), results, epsilon, true);
		/*for (int i = 0; i < contour.size(); i++)
		results[i] = contour[i];*/

		return results;
	}

	cv::Point2f CurveRightAngleSimplification::find_center(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3){
		double ax, ay, bx, by, cx, cy, x1, y11, dx1, dy1, x2, y2, dx2, dy2, ox, oy, dx, dy, radius;
		ax = p1.x; ay = p1.y;
		bx = p2.x; by = p2.y;
		cx = p3.x; cy = p3.y;

		x1 = (bx + ax) / 2;
		y11 = (by + ay) / 2;
		dy1 = bx - ax;
		dx1 = -(by - ay);
		x2 = (cx + bx) / 2;
		y2 = (cy + by) / 2;
		dy2 = cx - bx;
		dx2 = -(cy - by);
		ox = (y11 * dx1 * dx2 + x2 * dx1 * dy2 - x1 * dy1 * dx2 - y2 * dx1 * dx2) / (dx1 * dy2 - dy1 * dx2);
		oy = (ox - x1) * dy1 / dx1 + y11;
		return cv::Point2f(ox, oy);
	}

}