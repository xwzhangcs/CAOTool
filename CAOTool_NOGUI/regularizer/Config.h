#pragma once

#include <vector>
namespace regularizer {

	class Config {
	public:
		bool bUseIntra;
		float intraWeight;
		bool bUseInter;
		float interWeight;
		bool bUseRaOpt;
		float angle_threshold_RA;
		float raWeight;
		bool bUseParallelOpt;
		float angle_threshold_parallel;
		float parallelWeight;
		bool bUseSymmetryLineOpt;
		float symmetryIouThreshold;
		float symmetryWeight;
		bool bUseAccuracyOpt;
		float accuracyWeight;
		bool bUsePointSnapOpt;
		float pointDisThreshold;
		float pointWeight;
		bool bUseSegSnapOpt;
		float segDisThreshold;
		float segAngleThreshold;
		float segWeight;

	public:
		Config();
		Config(bool bUseIntra, float intraWeight, bool bUseInter, float interWeight, bool bUseRaOpt, float angle_threshold_RA, float raWeight, bool bUseParallelOpt, float angle_threshold_parallel, float parallelWeight,
			bool bUseSymmetryLineOpt, float symmetryIouThreshold, float symmetryWeight, bool bUseAccuracyOpt, float accuracyWeight, bool bUsePointSnapOpt, float pointDisThreshold, float pointWeight, bool bUseSegSnapOpt, float segDisThreshold, float segAngleThreshold, float segWeight);
	};
}

