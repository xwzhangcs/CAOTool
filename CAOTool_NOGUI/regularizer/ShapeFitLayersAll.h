#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dlib/optimization.h>
#include "../util/ContourUtils.h"
#include "../util/BuildingLayer.h"
#include "Config.h"

class ShapeFitLayersAll {
		
	typedef dlib::matrix<double, 0, 1> column_vector;
	typedef std::vector<std::vector<cv::Point2f>> layer_polygons;

	class BFGSSolver {
	private:
		std::vector<layer_polygons> target_layers_polygons;
		std::vector<layer_polygons> init_layers_polygons;
		std::vector<layer_polygons> layers_symmetry_lines;
		std::vector<std::pair<float, float>> layers_height;
		std::vector<std::pair<std::vector<int>, std::vector<int>>> tree_info;
		std::vector<std::vector<std::vector<bool>>> validity_layer_polygons;
		regularizer::Config config;

	public:
		BFGSSolver(const std::vector<layer_polygons>& target_layers_polygons, const std::vector<layer_polygons>& init_layers_polygons, const std::vector<layer_polygons>& layers_symmetry_lines, const std::vector<std::pair<float, float>>& layers_height, const std::vector<std::pair<std::vector<int>, std::vector<int>>>& tree_info, const std::vector<std::vector<std::vector<bool>>>& validity_layer_polygons, regularizer::Config& config) {
			this->target_layers_polygons = target_layers_polygons;
			this->init_layers_polygons = init_layers_polygons;
			this->layers_symmetry_lines = layers_symmetry_lines;
			this->tree_info = tree_info;
			this->config = config;
			this->validity_layer_polygons = validity_layer_polygons;
		}

		double operator() (const column_vector& arg) const {
			std::vector<layer_polygons> polygons;
			polygons.resize(init_layers_polygons.size());
			int start_index = 0;
			for (int k = 0; k < init_layers_polygons.size(); k++){
				polygons[k].resize(init_layers_polygons[k].size());
				for (int i = 0; i < init_layers_polygons[k].size(); i++){
					for (int j = 0; j < init_layers_polygons[k][i].size(); j++) {
						polygons[k][i].push_back(cv::Point2f(arg((j + start_index) * 2), arg((j + start_index) * 2 + 1)));
					}
					start_index += init_layers_polygons[k][i].size();
				}
			}
			try {
				float score = 0.0f;
				float layer_score = 0.0f;
				float polygon_score = 0.0f;
				int valid_layers = 0;
				int valid_layer_polygons = 0;
				for (int k = 0; k < init_layers_polygons.size(); k++){
					layer_score = 0.0f;
					valid_layer_polygons = 0;
					for (int i = 0; i < init_layers_polygons[k].size(); i++){
						polygon_score = 0.0f;
						if (init_layers_polygons[k][i].size() != 0){
							if (config.bUseRaOpt && validity_layer_polygons[k][i][0]){
								//std::cout << "use RA opt" << std::endl;
								float ra_score = util::calculateScoreRaOpt(polygons[k][i], init_layers_polygons[k][i], config.angle_threshold_RA);
								//std::cout << "During RA OPT,  socore is " << ra_score << std::endl;
								polygon_score += ra_score * config.raWeight * config.intraWeight;
							}
							// parallel opt function
							if (config.bUseParallelOpt && validity_layer_polygons[k][i][1]){
								//std::cout << "use Parallel opt" << std::endl;
								float parallel_score = util::calculateScoreParallelOpt(polygons[k][i], init_layers_polygons[k][i], config.angle_threshold_parallel);
								//std::cout << "During Parallel OPT,  socore is " << parallel_score << std::endl;
								polygon_score += parallel_score * config.parallelWeight * config.intraWeight;
							}

							// symmetry opt function
							if (config.bUseSymmetryLineOpt && validity_layer_polygons[k][i][2]){
								if (layers_symmetry_lines[k][i].size() != 0){
									//std::cout << "use Symmetry opt"<< std::endl;
									std::vector<cv::Point2f> polygon_symmetry;
									cv::Point2f a = layers_symmetry_lines[k][i][0];
									cv::Point2f b = layers_symmetry_lines[k][i][1];
									for (int j = 0; j < polygons[k][i].size(); j++) {
										polygon_symmetry.push_back(util::mirrorPoint(a, b, polygons[k][i][j]));
									}
									float iou = 0.0f;
									if (!util::isSimple(polygons[k][i]) || !util::isSimple(polygon_symmetry)){
										//std::cout << "image method" << std::endl;
										iou = util::calculateIOUbyImage(polygons[k][i], polygon_symmetry, 1000);
									}
									else{
										iou = util::calculateIOUbyCGAL(polygons[k][i], polygon_symmetry);
										//std::cout << "cgal method" << std::endl;
									}
									//std::cout << "During OPT, symmetry score is " << iou << std::endl;
									polygon_score += iou * config.symmetryWeight * config.intraWeight;
								}
								else{
									// no symmetry line for this polygon
								}
							}
							if (config.bUseAccuracyOpt)
							{
								//std::cout << "use Accuracy opt" << std::endl;
								float accuracy_score = 0.0f;
								if (!util::isSimple(polygons[k][i]) || !util::isSimple(target_layers_polygons[k][i])){
									//std::cout << "image method" << std::endl;
									accuracy_score += util::calculateIOUbyImage(polygons[k][i], target_layers_polygons[k][i], 1000);
								}
								else{
									//std::cout << "cgal method" << std::endl;
									accuracy_score += util::calculateIOUbyCGAL(polygons[k][i], target_layers_polygons[k][i]);
								}
								//std::cout << "During Accuracy OPT,  score is " << accuracy_score << std::endl;
								polygon_score += accuracy_score * config.accuracyWeight * config.intraWeight;
							}
							// optimization score for one polygon
							if (config.bUsePointSnapOpt && validity_layer_polygons[k][i][4]){
								float score_point = 0.0f;
								int valid_num = 0;
								//std::cout << "Point Snap Opt" << std::endl;
								for (int j = 0; j < tree_info[k].first.size(); j++){
									float score_tmp = util::calculateScorePointOpt(polygons[k][i], init_layers_polygons[k][i], polygons[tree_info[k].first[j]], init_layers_polygons[tree_info[k].first[j]], config.pointDisThreshold);
									if (abs(score_tmp) > 0.1){
										valid_num++;
										score_point += score_tmp;
									}
								}
								//std::cout << "valid_num is " << valid_num << std::endl;
								if (valid_num > 0){
									score_point = score_point / valid_num;
									//std::cout << "During Point OPT,  socore is " << score_point << std::endl;
								}
								polygon_score += score_point * config.pointWeight * config.interWeight;
							}
							if (config.bUseSegSnapOpt && validity_layer_polygons[k][i][5]){
								float score_seg = 0.0f;
								int valid_num = 0;
								//std::cout << "Seg Snap Opt" << std::endl;
								for (int j = 0; j < tree_info[k].first.size(); j++){
									float score_tmp = util::calculateScoreSegOpt(polygons[k][i], init_layers_polygons[k][i], polygons[tree_info[k].first[j]], init_layers_polygons[tree_info[k].first[j]], config.segDisThreshold, config.segAngleThreshold);
									if (abs(score_tmp) > 0.1){
										valid_num++;
										score_seg += score_tmp;
									}
								}
								if (valid_num > 0){
									score_seg = score_seg / valid_num;
									//std::cout << "During Seg OPT,  socore is " << score_seg << std::endl;
								}
								polygon_score += score_seg * config.segWeight * config.interWeight;
							}
						}
						for (int p = 0; p < 6; p++){
							if (validity_layer_polygons[k][i][p]){
								valid_layer_polygons++;
								break;
							}
						}
						layer_score += polygon_score;
					}
					//
					if (valid_layer_polygons == 0){
						layer_score = 0.0f;
					}
					else {
						valid_layers++;
						layer_score = layer_score / valid_layer_polygons;
					}
					//std::cout << "layer " << k << " score is " << layer_score << std::endl;
					score += layer_score;
				}
				if (valid_layers == 0)
					score = score;
				else
					score = score / valid_layers;
				//std::cout << "score is " << score << std::endl;
				return score;
			}
			catch (...) {
				std::cout << "exception" << std::endl;
				return 0;
			}
		}
	};

protected:
	ShapeFitLayersAll();
	~ShapeFitLayersAll();

public:
	static void fit(std::vector<std::shared_ptr<util::BuildingLayer>> & layers, std::vector<std::pair<int, int>>& layers_relationship, regularizer::Config config);
	static bool validRA(const std::vector<cv::Point2f>& polygon, bool bUseRaOpt, int ra_angle_threshold);
	static bool validParallel(const std::vector<cv::Point2f>& polygon, bool bUseParallelOpt, int parallel_angle_threshold);
	static bool validPointOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, float dis_threshold);
	static bool validSegOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, float dis_threshold, float angle_threshold);
	static bool validPointOpt(const util::Polygon& src_polygon, const std::vector<util::Polygon>& des_layer_polygons, float dis_threshold);
	static bool validSegOpt(const util::Polygon& src_polygon, const std::vector<util::Polygon>& des_layer_polygons, float dis_threshold, float angle_threshold);
};

