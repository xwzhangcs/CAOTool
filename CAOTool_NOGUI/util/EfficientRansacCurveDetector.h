#pragma once

#include "ContourUtils.h"

namespace util {

	class EfficientRansacCurveDetector {
	protected:
		EfficientRansacCurveDetector() {}

	public:
		static bool detect(const util::Polygon& polygons);
		static bool detect2(const std::vector<std::vector<util::Polygon>>& polygons);

	};

}