#include "PlyWriter.h"
#include <fstream>
#include "ContourUtils.h"

namespace util {
	
	namespace ply {

		void PlyWriter::write(const std::string& filename, double width, double height, double offset_x, double offset_y, double offset_z, double scale, const std::vector<std::shared_ptr<BuildingLayer>>& buildings) {
			std::map<Point3d, int> vertices_map;
			std::vector<Point3d> vertices;
			std::vector<std::vector<int>> faces;

			int num_polygons = 0;
			for (int i = 0; i < buildings.size(); i++) {
				num_polygons += writeBuilding(buildings[i], vertices_map, vertices, faces);
			}
			std::cout << "#polygons = " << num_polygons << std::endl;
			std::cout << "#vertices = " << vertices.size() << std::endl;

			std::ofstream out(filename, std::ios::binary);
			out << "ply" << std::endl;
			out << "format binary_little_endian 1.0" << std::endl;
			out << "element vertex " << vertices.size() << std::endl;
			out << "property float x" << std::endl;
			out << "property float y" << std::endl;
			out << "property float z" << std::endl;
			out << "element face " << faces.size() << std::endl;
			out << "property list uchar int vertex_indices" << std::endl;
			out << "end_header" << std::endl;
			
			// write vertices
			for (int i = 0; i < vertices.size(); i++) {
				float x = (vertices[i].x + width * 0.5 - 0.5) * scale + offset_x;
				float y = (vertices[i].y - height * 0.5 + 0.5) * scale + offset_y;
				float z = (vertices[i].z - 0.5) * scale + offset_z;
				out.write((char*)&x, sizeof(float));
				out.write((char*)&y, sizeof(float));
				out.write((char*)&z, sizeof(float));
			}

			// write faces
			for (int i = 0; i < faces.size(); i++) {
				uchar s = faces[i].size();
				out.write((char*)&s, sizeof(uchar));
				for (int j = 0; j < faces[i].size(); j++) {
					out.write((char*)&faces[i][j], sizeof(int));
				}
			}

			out.close();
		}

		int PlyWriter::writeBuilding(std::shared_ptr<BuildingLayer> building, std::map<Point3d, int>& vertices_map, std::vector<Point3d>& vertices, std::vector<std::vector<int>>& faces) {
			std::vector<std::vector<cv::Point2f>> polygons;

			for (auto& footprint : building->footprints) {
				if (footprint.primitive_shapes.size() == 0) {
					if (footprint.holes.size() == 0) {
						polygons = tessellate(footprint.contour);
					}
					else {
						polygons = tessellate(footprint.contour, footprint.holes);
					}
				}
				else {
					for (int i = 0; i < footprint.primitive_shapes.size(); i++) {
						std::vector<cv::Point2f> points = footprint.primitive_shapes[i]->getActualPoints();
						polygons.push_back(points);
					}
				}
			}		

			for (auto polygon : polygons) {
				util::clockwise(polygon);

				std::vector<int> bottom_face(polygon.size());
				std::vector<int> top_face(polygon.size());

				for (int i = 0; i < polygon.size(); i++) {
					Point3d pt_bottom(polygon[i].x, polygon[i].y, building->bottom_height);
					Point3d pt_top(polygon[i].x, polygon[i].y, building->top_height);

					int index_bottom;
					if (vertices_map.find(pt_bottom) == vertices_map.end()) {
						index_bottom = vertices.size();
						vertices.push_back(pt_bottom);
						vertices_map[pt_bottom] = index_bottom;
					}
					else {
						index_bottom = vertices_map[pt_bottom];
					}
					bottom_face[i] = index_bottom;

					int index_top;
					if (vertices_map.find(pt_top) == vertices_map.end()) {
						index_top = vertices.size();
						vertices.push_back(pt_top);
						vertices_map[pt_top] = index_top;
					}
					else {
						index_top = vertices_map[pt_top];
					}
					top_face[polygon.size() - i - 1] = index_top;
				}

				faces.push_back(bottom_face);
				faces.push_back(top_face);
			}
			
			/*
			// side faces
			util::Ring polygon = building->footprint.contour.getActualPoints();
			polygon.counterClockwise();
			for (int i = 0; i < polygon.size(); i++) {
				int next = (i + 1) % polygon.size();
				std::vector<Point3d> pts(4);
				pts[0] = Point3d(polygon[i].x, polygon[i].y, building->bottom_height);
				pts[1] = Point3d(polygon[next].x, polygon[next].y, building->bottom_height);
				pts[2] = Point3d(polygon[next].x, polygon[next].y, building->top_height);
				pts[3] = Point3d(polygon[i].x, polygon[i].y, building->top_height);
				
				std::vector<int> indices(4);
				for (int i = 0; i < 4; i++) {
					indices[i] = findClosestVertexIndex(pts[i], vertices_map);
				}
				
				faces.push_back(indices);
			}

			// side faces of holes
			for (int hid = 0; hid < building->footprint.holes.size(); hid++) {
				Ring hole = building->footprint.holes[hid].getActualPoints();
				hole.clockwise();
				for (int i = 0; i < hole.size(); i++) {
					int next = (i + 1) % hole.size();
					std::vector<Point3d> pts(4);
					pts[0] = Point3d(hole[i].x, hole[i].y, building->bottom_height);
					pts[1] = Point3d(hole[next].x, hole[next].y, building->bottom_height);
					pts[2] = Point3d(hole[next].x, hole[next].y, building->top_height);
					pts[3] = Point3d(hole[i].x, hole[i].y, building->top_height);

					std::vector<int> indices(4);
					for (int i = 0; i < 4; i++) {
						indices[i] = findClosestVertexIndex(pts[i], vertices_map);
					}

					faces.push_back(indices);
				}
			}
			*/

			/////////////////////////////////////////////////////
			// Previously used the contour polygon and holes to generate side faces,
			// but updated to use the primitive shapes for generating side faces
			for (auto polygon : polygons) {
				util::counterClockwise(polygon);

				for (int i = 0; i < polygon.size(); i++) {
					int next = (i + 1) % polygon.size();
					std::vector<Point3d> pts(4);
					pts[0] = Point3d(polygon[i].x, polygon[i].y, building->bottom_height);
					pts[1] = Point3d(polygon[next].x, polygon[next].y, building->bottom_height);
					pts[2] = Point3d(polygon[next].x, polygon[next].y, building->top_height);
					pts[3] = Point3d(polygon[i].x, polygon[i].y, building->top_height);

					std::vector<int> indices(4);
					for (int k = 0; k < 4; k++) {
						indices[k] = findClosestVertexIndex(pts[k], vertices_map);
					}

					faces.push_back(indices);
				}
			}

			int num_polygons = polygons.size();
			for (auto child : building->children) {
				num_polygons += writeBuilding(child, vertices_map, vertices, faces);
			}

			return num_polygons;
		}

		int PlyWriter::findClosestVertexIndex(const Point3d& p, const std::map<Point3d, int>& points) {
			if (points.find(p) != points.end()) {
				return points.at(p);
			}
			else {
				int ans = -1;

				float min_length = std::numeric_limits<float>::max();
				for (auto it = points.begin(); it != points.end(); it++) {
					Point3d a = it->first;
					float length = (a - p).length();
					if (length < min_length) {
						min_length = length;
						ans = it->second;
					}
				}
				return ans;
			}

		}

	}

}