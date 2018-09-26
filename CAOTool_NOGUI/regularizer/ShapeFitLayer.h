#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dlib/optimization.h>
#include "../util/ContourUtils.h"
#include "Config.h"

class ShapeFitLayer {
		
	typedef dlib::matrix<double, 0, 1> column_vector;

	class BFGSSolver {
	private:
		std::vector<std::vector<cv::Point2f>> target_polygons;
		std::vector<std::vector<cv::Point2f>> init_polygons;
		bool bUseRaOpt;
		float angle_threshold_RA;
		float raWeight;
		bool bUseParallelOpt;
		float angle_threshold_parallel;
		float parallelWeight;
		bool bUseSymmetryLineOpt;
		std::vector<std::vector<cv::Point2f>> symmetry_lines;
		float symmetryWeight;
		bool bUseAccuracy;
		float accuracyWeight;

	public:
		BFGSSolver(const std::vector<std::vector<cv::Point2f>>& target_polygons, const std::vector<std::vector<cv::Point2f>>& init_polygons, bool bUseRaOpt, float angle_threshold_RA, float raWeight, bool bUseParallelOpt, float angle_threshold_parallel, float parallelWeight, bool bUseSymmetryLineOpt, const std::vector<std::vector<cv::Point2f>>& symmetry_lines, float symmetryWeight, bool bUseAccuracy, float accuracyWeight) {
			this->target_polygons = target_polygons;
			this->init_polygons = init_polygons;
			this->bUseRaOpt = bUseRaOpt;
			this->angle_threshold_RA = angle_threshold_RA;
			this->raWeight = raWeight;
			this->bUseParallelOpt = bUseParallelOpt;
			this->angle_threshold_parallel = angle_threshold_parallel;
			this->parallelWeight = parallelWeight;
			this->bUseSymmetryLineOpt = bUseSymmetryLineOpt;
			this->symmetry_lines = symmetry_lines;
			this->symmetryWeight = symmetryWeight;
			this->bUseAccuracy = bUseAccuracy;
			this->accuracyWeight = accuracyWeight;		
		}

		double operator() (const column_vector& arg) const {
			std::vector<std::vector<cv::Point2f>> polygons;
			polygons.resize(init_polygons.size());
			int start_index = 0;
			for (int i = 0; i < init_polygons.size(); i++){
				for (int j = 0; j < init_polygons[i].size(); j++) {
					polygons[i].push_back(cv::Point2f(arg((j + start_index) * 2), arg((j + start_index) * 2 + 1)));
				}
				start_index += init_polygons[i].size();
			}
			try {
				float score = 0.0f;
				int valid_polygons = 0;
				for (int i = 0; i < init_polygons.size(); i++){
					if (init_polygons[i].size() != 0){
						// RA opt function
						if (bUseRaOpt){
							//std::cout << "use RA opt" << std::endl;
							float ra_score = util::calculateScoreRaOpt(polygons[i], init_polygons[i], angle_threshold_RA);
							//std::cout << "During OPT, ra_score is " << ra_score << std::endl;
							score += ra_score * raWeight;
							//std::cout << "score is " << score <<std::endl;
						}
						// parallel opt function
						if (bUseParallelOpt){
							//std::cout << "use Parallel opt" << std::endl;
							float parallel_score = util::calculateScoreParallelOpt(polygons[i], init_polygons[i], angle_threshold_parallel);
							//std::cout << "During OPT, parallel_score is " << parallel_score << std::endl;
							score += parallel_score * parallelWeight;
						}

						// symmetry opt function
						if (bUseSymmetryLineOpt){
							if (symmetry_lines[i].size() != 0){
								//std::cout << "symmetry polygon is " << i << std::endl;
								std::vector<cv::Point2f> polygon_symmetry;
								cv::Point2f a = symmetry_lines[i][0];
								cv::Point2f b = symmetry_lines[i][1];
								for (int j = 0; j < polygons[i].size(); j++) {
									polygon_symmetry.push_back(util::mirrorPoint(a, b, polygons[i][j]));
								}
								float iou = 0.0f;
								if (!util::isSimple(polygons[i]) || !util::isSimple(polygon_symmetry)){
									//std::cout << "image method" << std::endl;
									iou = util::calculateIOUbyImage(polygons[i], polygon_symmetry, 1000);
								}
								else{
									iou = util::calculateIOUbyCGAL(polygons[i], polygon_symmetry);
									//std::cout << "cgal method" << std::endl;
								}
								//std::cout << "During OPT, IOU is " << iou << std::endl;
								score += iou * symmetryWeight;
							}
							else{
								// no symmetry line for this polygon
							}
						}
						if (bUseAccuracy)
						{
							//std::cout << "use Accuracy opt" << std::endl;
							float accuracy_score = 0.0f;
							if (!util::isSimple(polygons[i]) || !util::isSimple(target_polygons[i])){
								//std::cout << "image method" << std::endl;
								accuracy_score = util::calculateIOUbyImage(polygons[i], target_polygons[i], 1000);
							}
							else{
								//std::cout << "cgal method" << std::endl;
								accuracy_score = util::calculateIOUbyCGAL(polygons[i], target_polygons[i]);
							}
							//std::cout << "During OPT, IOU is " << accuracy_score << std::endl;
							score += accuracy_score * accuracyWeight;
						}
						if (bUseSymmetryLineOpt && !bUseRaOpt && !bUseParallelOpt && !bUseAccuracy){
							if (symmetry_lines[i].size() != 0)
								valid_polygons++;
						}
						else
							valid_polygons++;
					}
				}
				if (valid_polygons > 0)
					score = score / valid_polygons;
				//std::cout << " score is " << score << std::endl;
				//std::cout << "---------------- " << std::endl;
				return score;
			}
			catch (...) {
				std::cout << "exception" << std::endl;
				return 0;
			}
		}
	};

protected:
	ShapeFitLayer();
	~ShapeFitLayer();

public:
	static std::vector<util::Polygon> fit(const std::vector<util::Polygon>& polygons, const std::vector<util::Polygon>& ini_points, regularizer::Config config);
	static bool validRAorParallel(const std::vector<cv::Point2f>& polygon, bool bUseRaOpt, int ra_angle_threshold, bool bUseParallelOpt, int parallel_angle_threshold);
};

