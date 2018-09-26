#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "BuildingLayer.h"

namespace util {

	namespace ply {

		class Point3d {
		public:
			double x;
			double y;
			double z;

		public:
			Point3d() {}
			Point3d(double x, double y, double z) : x(x), y(y), z(z) {}

			Point3d operator-(const Point3d& p2) {
				return Point3d(x - p2.x, y - p2.y, z - p2.z);
			}

			float length() {
				return std::sqrt(x * x + y * y + z * z);
			}

			friend bool operator<(const Point3d& p1, const Point3d& p2) {
				return std::tie(p1.x, p1.y, p1.z) < std::tie(p2.x, p2.y, p2.z);
			}
		};


		class PlyWriter {
		protected:
			PlyWriter() {}

		public:
			static void write(const std::string& filename, double width, double height, double offset_x, double offset_y, double offset_z, double scale, const std::vector<std::shared_ptr<BuildingLayer>>& buildings);

		private:
			static int writeBuilding(std::shared_ptr<BuildingLayer> building, std::map<Point3d, int>& vertices_map, std::vector<Point3d>& vertices, std::vector<std::vector<int>>& faces);
			static int findClosestVertexIndex(const Point3d& p, const std::map<Point3d, int>& points);
		};

	}

}