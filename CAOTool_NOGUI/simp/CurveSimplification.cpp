#include "CurveSimplification.h"
#include "../util/ContourUtils.h"

namespace simp {

	/**
	* Simplify the footprint of the layer.
	*
	* @param polygon			contour polygon of the layer
	* @param epsilon			epsilon parameter for DP method
	* @param curve_threshold	maximum deviation of the point from the arc
	* @param orientation		principle orientation of the contour in radian
	* @param min_hole_ratio		hole will be removed if its area ratio to the contour is less than this threshold
	* @return					simplified footprint
	*/
	util::Polygon CurveSimplification::simplify(const util::Polygon& polygon, float epsilon, float curve_threshold, float orientation, float min_hole_ratio) {
		util::Polygon ans;

		// create a slice image from the input polygon
		cv::Rect bbox = util::boundingBox(polygon.contour.points);
		cv::Mat_<uchar> img;
		util::createImageFromPolygon(bbox.width, bbox.height, polygon, cv::Point2f(-bbox.x, -bbox.y), img);
		
		std::vector<util::Polygon> polygons = findContours(img, epsilon, curve_threshold);
		if (polygons.size() == 0) throw "No building is found.";

		// tranlsate (bbox.x, bbox.y)
		polygons[0].translate(bbox.x, bbox.y);

		decomposePolygon(polygons[0], ans, epsilon, curve_threshold);
		if (ans.contour.size() < 3){
			throw "No building is found.";
		}
		return ans;
	}

	void CurveSimplification::decomposePolygon(util::Polygon input, util::Polygon& polygon, float epsilon, float curve_threshold) {
		if (input.holes.size() == 0){
			std::vector<cv::Point2f> contour;
			contour.resize(input.contour.size());
			for (int i = 0; i < input.contour.size(); i++)
				contour[i] = input.contour[i];
			util::Polygon output;
			bool bContainCurve = false;
			if (contour.size() > 100){
				bContainCurve = approxContour(contour, output, epsilon, curve_threshold);
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
			//std::cout << "contour size " << contour.size() << std::endl;
			util::Polygon output;
			bool bContainCurve = false;
			if (contour.size() > 100){
				bContainCurve = approxContour(contour, output, epsilon, curve_threshold);
			}
			//std::cout << "after approxContour" << std::endl;
			if (bContainCurve)
			{
				//std::cout << "bContainCurve" << std::endl;
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
				if (contour.size() > 100){
					bContainCurve = approxContour(contour, output, epsilon, curve_threshold);
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
	std::vector<util::Polygon> CurveSimplification::findContours(const cv::Mat_<uchar>& img, float epsilon, float curve_threshold) {
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
		cv::findContours(padded, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

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
	bool CurveSimplification::approxContour(std::vector<cv::Point2f>& input, util::Polygon &output, float epsilon, float curve_threshold){
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
		for (int i = 0; i < input.size();){
			// initialize variables
			points.clear();
			init_points = cv::Point3d(20, 20, 10);
			dis = (int)(percentage * input.size());
			if (dis < 50 && input.size() > 50)
				dis = 50;
			if (dis > 100)
				dis = 100;
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
					dis++;
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
		int start_init = 0;
		for (int i = 0; i < contour_points_type.size(); i++){
			if (contour_points_type[i] == 1){
				start_init = i;
				break;
			}
		}
		for (int i = start_init; i < start_init + contour_points_type.size();){
			simplified_curve_tmp.clear();
			bool bCurve = false;
			while (contour_points_type[i % contour_points_type.size()] == 2 && i < start_init + contour_points_type.size()){
				bCurve = true;
				simplified_curve_tmp.push_back(clean_contour[i % contour_points_type.size()]);
				i++;
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


				if (abs(angle_start_end) >= 60 && radius > 30){
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
			// generate output polygon and decomposePolygon here
			output.contour.resize(final_contour.size());
			
			int start_index = 0;
			bool bfind_start = false;
			int regular_points = 0;
			// find the index of starting point
			for (int i = 0; i < final_contour.size(); i++){
				//std::cout << "final contour point is (" << final_contour[i].x << ", " << final_contour[i].y<<")" << std::endl;
				output.contour[i] = final_contour[i];
				if (type_final_contour[i] == 1){
					if (!bfind_start)
						start_index = i;
					bfind_start = true;
				}
			}
			// check whether the polygon is simple
			if (!util::isSimple(output.contour))
				return false;
			// The transfomration matrix should be same for the external contour and the internal holes
			// because for OpenCV simplification, transformation is just to trasnform from OpenCV image coordinates
			// to the world coordinate system.
			output.mat = output.contour.mat;
			// find primitives
			output.primitive_shapes.clear();
			// a. curve primitives
			//
			int num_curves = 0;
			int tmp = 0;
			bool bCurve = false;
			cv::Point2f center;
			float radius;
			std::vector<cv::Point2f> curve_poitns;
			for (int i = start_index; i < start_index + final_contour.size();){
				tmp = i % final_contour.size();

				if (type_final_contour[tmp] == 1){
					bCurve = false;
					i++;
				}
				curve_poitns.clear();
				int curve_start_index = 0;
				int curve_end_index = 0;
				while (type_final_contour[tmp] == 2 && i < start_index + final_contour.size()){
					if (!bCurve){
						center = cv::Point2f(final_contour_curve[tmp].x, final_contour_curve[tmp].y);
						radius = final_contour_curve[tmp].z;
						curve_start_index = tmp;
					}
					curve_poitns.push_back(final_contour[tmp]);
					i++;
					tmp = i % final_contour.size();
					curve_end_index = tmp;
					bCurve = true;
				}
				if (bCurve){
					center = find_center(curve_poitns[0], curve_poitns[curve_poitns.size() / 2], curve_poitns[curve_poitns.size() - 1]);
					cv::Point2f start = (curve_poitns[0] - center);
					cv::Point2f mid = (curve_poitns[curve_poitns.size() / 2] - center);
					cv::Point2f end = (curve_poitns[curve_poitns.size() - 1] - center);
					cv::Point2f x_axis(1.0, 0);
					// check whether it's a concave curve 
					std::vector<cv::Point2f> end_points;
					end_points.push_back(start + center);
					end_points.push_back(end + center);
					if (concaveCurve(end_points, output) && curve_poitns.size() < final_contour.size()){
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
					float theta_start = compute_angle(x_axis, start);
					float theta_start_end = compute_interval(start, mid, end);
					float theta_end = theta_start + theta_start_end;
					radius = cv::norm(start);
				
					boost::shared_ptr<util::PrimitiveCurve> pol = boost::shared_ptr<util::PrimitiveCurve>(new util::PrimitiveCurve(output.mat, theta_start, theta_end, center, radius));
					output.primitive_shapes.push_back(pol);
					num_curves++;
				}
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
			//std::cout << "num_curves is " << num_curves << std::endl;
			return bContainCurve;
		}
		else
			return bContainCurve;
	}

	bool CurveSimplification::concaveCurve(std::vector<cv::Point2f>& end_points, util::Polygon &polygon){
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
		if (outside > 2)
			return true;
		else
			return false;

	}

	cv::Point2f CurveSimplification::find_center(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3){
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

	cv::Point3f CurveSimplification::optimizeByBFGS(const std::vector<cv::Point2d>& points, cv::Point3d init_points) {
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

	float CurveSimplification::compute_angle(cv::Point2d a, cv::Point2d b){
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

	float CurveSimplification::compute_abs_angle(cv::Point2d a, cv::Point2d b){
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

	float CurveSimplification::compute_interval(cv::Point2d start, cv::Point2d mid, cv::Point2d end){
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

	bool CurveSimplification::valid_curve(double threshold, const std::vector<cv::Point2d>& points, cv::Point2d center, double radius, cv::Rect bbox){
		bool bValid = true;
		// compute max_error
		double max_error = 0;
		for (int i = 0; i < points.size(); i++){
			double tmp = abs((cv::norm(center - points.at(i)) - abs(radius)));
			//tmp= glm::pow((glm::length2(center - points.at(i)) - radius*radius), 2);
			if (tmp > max_error)
				max_error = tmp;
		}
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
		/*if (angle_v2 < 5)
			return false;
		if (angle_v3 < 5)
			return false;
		if (angle_v4 < 5)
			return false;*/
		// center comparison
		{

		}
		// radius comparison
		if (radius > 0.5 * diaginal)
			return false;
		/*if (radius < )*/
		return true;
	}

}