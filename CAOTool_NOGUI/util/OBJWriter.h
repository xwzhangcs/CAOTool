#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "BuildingLayer.h"
#include "VoxelBuilding.h"

namespace util {

	namespace obj {

		class Vertex {
		public:
			cv::Point3f position;
			cv::Point2f tex_coord;
			cv::Point3f normal;

		public:
			Vertex() {}
			Vertex(const cv::Point3f& position, const cv::Point3f& normal) : position(position), normal(normal) {}
			Vertex(const cv::Point3f& position, const cv::Point2f& tex_coord, const cv::Point3f& normal) : position(position), tex_coord(tex_coord), normal(normal) {}
		};

		class Face {
		public:
			std::vector<Vertex> vertices;
			std::string texture;
			cv::Point3f color;
			bool texture_enabled;

		public:
			Face() {}
			Face(const std::vector<Vertex>& vertices, const std::string& texture) : vertices(vertices), texture(texture), texture_enabled(true) {}
			Face(const std::vector<Vertex>& vertices, const cv::Point3f& color) : vertices(vertices), color(color), texture_enabled(false) {}
		};

		class Material {
		public:
			enum { TYPE_COLOR = 0, TYPE_TEXTURE };

		private:
			int type;
			cv::Point3f color;
			std::string texture;

		public:
			Material() : type(TYPE_COLOR) {}
			Material(const cv::Point3f& color) : type(TYPE_COLOR), color(color) {}
			Material(const std::string& texture) : type(TYPE_TEXTURE), texture(texture) {}

			bool equals(const Material& other);
			std::string to_string();

			friend bool operator<(const Material& m1, const Material& m2) {
				return std::tie(m1.type, m1.color.x, m1.color.y, m1.color.z, m1.texture) < std::tie(m2.type, m2.color.x, m2.color.y, m2.color.z, m2.texture);
			}
		};

		class OBJWriter {
		protected:
			OBJWriter() {}

		public:
			static void write(const std::string& filename, double width, double height, double offset_x, double offset_y, double offset_z, double scale, const std::vector<std::shared_ptr<BuildingLayer>>& buildings);
			static void writeVoxels(const std::string& filename, double width, double height, double offset_x, double offset_y, double offset_z, double scale, const std::vector<VoxelBuilding>& voxel_buildings);

		private:
			static void writeBuilding(std::shared_ptr<BuildingLayer> building, double scale, const cv::Point3f& color, const std::string& facade_texture, const std::string& roof_texture, std::vector<Face>& faces);
			static void createFace(const std::vector<cv::Point2f>& coords, double z, double h, float floor_tile_width, float floor_tile_height, const cv::Point3f& color, const std::string& facade_texture, std::vector<Face>& faces);
			static double dotProductBetweenThreePoints(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c);
			static double getLength(const std::vector<cv::Point2f>& points);
			static void writeVoxelNode(std::shared_ptr<VoxelNode> voxel_node, double scale, const cv::Point3f& color, std::vector<Face>& faces);
		};

	}
}
