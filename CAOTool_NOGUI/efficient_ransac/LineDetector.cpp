#include "LineDetector.h"
#include "EfficientRANSACUtil.h"

namespace efficient_ransac {

	Line::Line() {
		_right_angle = false;
	}

	Line::Line(int index, const cv::Point2f& point, const cv::Point2f& dir) : _point(point), _dir(dir / cv::norm(dir)) {
		_start_index = index;
		_start_point = point;
		_end_index = index;
		_end_point = point;
		_right_angle = false;
	}

	void LineDetector::detect(std::vector<Point>& polygon, int num_iter, int min_points, float max_error, float cluster_epsilon, float min_length, std::vector<float>& principal_angles, float angle_threshold, std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>>& lines) {
		lines.clear();
		//srand(1);
		boost::random::mt19937 gen;
		// generate a random number from 0 ~ 1000000
		boost::random::uniform_int_distribution<> dist(0, 1000000);
		int N = polygon.size();
		if (N < min_points) return;

		std::vector<cv::Point2f> normals(polygon.size());
		for (int i = 0; i < N; i++) {
			int prev = (i - 3 + N) % N;
			int next = (i + 3) % N;
			cv::Point2f dir = polygon[next].pos - polygon[prev].pos;
			dir /= cv::norm(dir);
			polygon[i].normal = cv::Point2f(dir.y, -dir.x);
		}

		int N2 = N;
		while (N2 < cluster_epsilon) N2 += N;

		// initialize the unused list
		std::vector<int> unused_list;
		for (int i = 0; i < N; i++) {
			if (!polygon[i].used) unused_list.push_back(i);
		}

		while (true) {
			int max_num_points = 0;
			std::shared_ptr<Line> best_line;
			int best_index1;

			for (int iter = 0; iter < num_iter && unused_list.size() >= 2; iter++) {
				// randomly sample index1 as a first point, and then, sample another point that are close to the first one
				int index1 = -1;
				int index2 = -1;
				for (int iter2 = 0; iter2 < num_iter && unused_list.size() >= 2; iter2++) {
					//index1 = unused_list[rand() % unused_list.size()];
					//index2 = (int)(index1 + rand() % (int)(cluster_epsilon * 2 + 1) - cluster_epsilon + N2) % N;
					index1 = unused_list[dist(gen) % unused_list.size()];
					index2 = (int)(index1 + dist(gen) % (int)(cluster_epsilon * 2 + 1) - cluster_epsilon + N2) % N;
					if (index2 == index1 || polygon[index2].used) continue;
					break;
				}

				if (index1 == -1 || index2 == -1) continue;

				// calculate the direction
				std::shared_ptr<Line> line = std::shared_ptr<Line>(new Line(index1, polygon[index1].pos, polygon[index2].pos - polygon[index1].pos));

				// cancel this proposal if the normal is too different
				if (std::abs(line->dir().dot(normals[index1])) > 0.1f) continue;

				// snap the orientation to the closest principal orientation
				if (principal_angles.size() > 0) {
					float angle = std::atan2(line->dir().y, line->dir().x);

					float best_angle;
					float min_diff = std::numeric_limits<float>::max();
					for (int i = 0; i < principal_angles.size(); i++) {
						float diff = angle_difference(principal_angles[i], angle);
						if (diff < min_diff) {
							min_diff = diff;
							best_angle = principal_angles[i];
						}
					}

					if (min_diff <= angle_threshold) {
						line->setDir(cv::Point2f(std::cos(best_angle), std::sin(best_angle)));
					}
				}

				// check whether the points are supporting this line
				std::vector<cv::Point2f> supporting_points;
				std::vector<int> supporting_indices;
				int prev = 0;
				for (int i = 1; i < N && i - prev < cluster_epsilon; i++) {
					int idx = (index1 - i + N) % N;
					if (polygon[idx].used) break;
					if (line->distance(polygon[idx].pos) < max_error) {
						prev = i;
						supporting_points.push_back(polygon[idx].pos);
						supporting_indices.push_back(index1 - i);
					}
				}
				std::reverse(supporting_points.begin(), supporting_points.end());
				std::reverse(supporting_indices.begin(), supporting_indices.end());
				prev = 0;
				for (int i = 0; i < N && i - prev < cluster_epsilon; i++) {
					int idx = (index1 + i) % N;
					if (polygon[idx].used) break;
					if (line->distance(polygon[idx].pos) < max_error) {
						prev = i;
						supporting_points.push_back(polygon[idx].pos);
						supporting_indices.push_back(index1 + i);
					}
				}

				// calculate two end points of the line segment
				line->setSupportingPoints(supporting_points, supporting_indices);
				if (line->length() < min_length) continue;

				if (line->points().size() > max_num_points) {
					max_num_points = line->points().size();
					best_line = line;
					best_index1 = index1;
				}
			}

			// if the best detected line does not have enough supporing points, terminate the algorithm.
			if (max_num_points < min_points || max_num_points == 0) break;
			
			// update used flag
			for (int i = best_line->startIndex(); i <= best_line->endIndex(); i++) {
				int idx = (i + N) % N;
				polygon[idx].used = true;
			}

			// update used list
			for (int i = unused_list.size() - 1; i >= 0; i--) {
				if (polygon[unused_list[i]].used) unused_list.erase(unused_list.begin() + i);
			}

			lines.push_back({ best_line->startIndex(), best_line });

		}
	}
}
