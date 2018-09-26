#include "OBJWriter.h"
#include <fstream>
#include <sstream>
#include <map>
#include "ContourUtils.h"

namespace util {

	namespace obj {

		bool Material::equals(const Material& other) {
			if (type != other.type) return false;
			if (type == TYPE_COLOR) {
				if (color != other.color) return false;
			}
			else if (type == TYPE_TEXTURE) {
				if (texture != other.texture) return false;
			}

			return true;
		}

		std::string Material::to_string() {
			std::stringstream ss;

			if (type == TYPE_COLOR) {
				ss << "Ka " << color.x << " " << color.y << " " << color.z << std::endl;
				ss << "Kd " << color.x << " " << color.y << " " << color.z << std::endl;
				ss << "Ks " << color.x << " " << color.y << " " << color.z << std::endl;
				ss << "Ke " << color.x << " " << color.y << " " << color.z << std::endl;
				ss << "d 1" << std::endl;
				ss << "Ns 20" << std::endl;
				ss << "illum 2" << std::endl;
			}
			else if (type == TYPE_TEXTURE) {
				ss << "Ka 1 1 1" << std::endl;
				ss << "Kd 1 1 1" << std::endl;
				ss << "Ks 1 1 1" << std::endl;
				ss << "Ke 1 1 1" << std::endl;
				ss << "d 1" << std::endl;
				ss << "Ns 20" << std::endl;
				ss << "illum 2" << std::endl;
				//ss << "map_Ka " << texture << std::endl;
				ss << "map_Kd " << texture << std::endl;
				//ss << "map_Ks " << texture << std::endl;
				//ss << "map_Ns " << texture << std::endl;
				//ss << "map_d " << texture << std::endl;
			}

			return ss.str();
		}
		
		void OBJWriter::write(const std::string& filename, double width, double height, double offset_x, double offset_y, double offset_z, double scale, const std::vector<std::shared_ptr<BuildingLayer>>& buildings) {
			std::vector<Vertex> vertices;
			std::vector<Face> faces;

			for (int i = 0; i < buildings.size(); i++) {
				// randomly select a texture file for the facade
				srand(time(NULL));
				int facade_texture_id = rand() % 13;

				// select a color based on the texture
				cv::Point3f color;
				if (facade_texture_id == 0) {
					color = cv::Point3f(0.741, 0.623, 0.490);
				}
				else if (facade_texture_id == 1) {
					color = cv::Point3f(0.792, 0.768, 0.721);
				}
				else if (facade_texture_id == 2) {
					color = cv::Point3f(0.772, 0.709, 0.647);
				}
				else if (facade_texture_id == 3) {
					color = cv::Point3f(0.674, 0.639, 0.572);
				}
				else if (facade_texture_id == 4) {
					color = cv::Point3f(0.776, 0.819, 0.843);
				}
				else if (facade_texture_id == 5) {
					color = cv::Point3f(0.882, 0.882, 0.874);
				}
				else if (facade_texture_id == 6) {
					color = cv::Point3f(0.811, 0.811, 0.803);
				}
				else if (facade_texture_id == 7) {
					color = cv::Point3f(0.850, 0.831, 0.815);
				}
				else if (facade_texture_id == 8) {
					color = cv::Point3f(0.811, 0.831, 0.843);
				}
				else if (facade_texture_id == 9) {
					color = cv::Point3f(0.784, 0.721, 0.623);
				}
				else if (facade_texture_id == 10) {
					color = cv::Point3f(0.494, 0.388, 0.321);
				}
				else if (facade_texture_id == 11) {
					color = cv::Point3f(0.756, 0.780, 0.772);
				}
				else if (facade_texture_id == 12) {
					color = cv::Point3f(0.764, 0.768, 0.788);
				}

				std::stringstream facade_texture;
				facade_texture << "textures/window_tile" << facade_texture_id << ".png";
				
				writeBuilding(buildings[i], scale, color, facade_texture.str(), "textures/roof_texture.png", faces);
			}

			std::ofstream file(filename);

			int index = filename.rfind('.');
			std::string mat_filename;
			if (index >= 0) {
				mat_filename = filename.substr(0, index) + ".mtl";
			}
			index = mat_filename.rfind('/');
			std::ofstream mat_file(mat_filename);
			if (index >= 0) {
				mat_filename = mat_filename.substr(index + 1);
			}

			file << "mtllib " << mat_filename << std::endl;
			file << std::endl;

			// grouping the faces based on their materials
			int positionId = 1;
			int texCoordId = 1;
			int materialId = 1;
			Material material;
			std::map<Material, std::vector<Face>> mapped_faces;
			for (auto& face : faces) {
				Material new_material;
				if (face.texture_enabled) {
					new_material = Material(face.texture);
				}
				else {
					new_material = Material(face.color);
				}
				mapped_faces[new_material].push_back(face);
			}

			// output vertex coordinates
			file << "# List of geometric vertices" << std::endl;
			for (auto it = mapped_faces.begin(); it != mapped_faces.end(); it++) {
				for (auto& face : it->second) {
					for (auto& vertex : face.vertices) {
						float x = (vertex.position.x + width * 0.5 - 0.5) * scale + offset_x;
						float y = (vertex.position.y - height * 0.5 + 0.5) * scale + offset_y;
						float z = (vertex.position.z - 0.5) * scale + offset_z;
						file << std::setprecision(20) << "v " << x << " " << y << " " << z << std::endl;
					}
				}
			}
			file << std::endl;

			// output texture coordinates
			file << "# List of texture coordinates" << std::endl;
			for (auto it = mapped_faces.begin(); it != mapped_faces.end(); it++) {
				for (auto& face : it->second) {
					if (face.texture_enabled) {
						for (auto& vertex : face.vertices) {
							file << "vt " << vertex.tex_coord.x << " " << vertex.tex_coord.y << std::endl;
						}
					}
				}
			}
			file << std::endl;

			// output normal vectors
			file << "# List of vertex normals" << std::endl;
			for (auto it = mapped_faces.begin(); it != mapped_faces.end(); it++) {
				for (auto& face : it->second) {
					for (auto& vertex : face.vertices) {
						file << "vn " << vertex.normal.x << " " << vertex.normal.y << " " << vertex.normal.z << std::endl;
					}
				}
			}
			file << std::endl;

			// output face information
			for (auto it = mapped_faces.begin(); it != mapped_faces.end(); it++) {
				Material material = it->first;

				mat_file << "newmtl Material" << materialId << std::endl;
				mat_file << material.to_string() << std::endl;

				file << std::endl;
				file << "usemtl Material" << materialId << std::endl;
				materialId++;

				for (auto& face : it->second) {
					file << "f";
					for (int k = 0; k < face.vertices.size(); k++) {
						file << " " << positionId;

						file << "/";
						if (face.texture_enabled) {
							file << texCoordId++;
						}

						file << "/" << positionId++;
					}
					file << std::endl;

				}
			}

			file.close();
		}

		void OBJWriter::writeVoxels(const std::string& filename, double width, double height, double offset_x, double offset_y, double offset_z, double scale, const std::vector<VoxelBuilding>& voxel_buildings) {
			std::vector<Vertex> vertices;
			std::vector<Face> faces;

			for (auto& voxel_building : voxel_buildings) {
				for (int z = 0; z < voxel_building.node_stack.size(); z++) {
					for (auto voxel_node : voxel_building.node_stack[z]) {
						writeVoxelNode(voxel_node, scale, cv::Point3f(1, 1, 1), faces);
					}
				}
			}

			std::ofstream file(filename);

			// grouping the faces based on their materials
			int positionId = 1;

			// output vertex coordinates
			file << "# List of geometric vertices" << std::endl;
			for (auto& face : faces) {
				for (auto& vertex : face.vertices) {
					file << std::setprecision(20) << "v " << (vertex.position.x + width * 0.5 - 0.5) * scale + offset_x << " " << (vertex.position.y - height * 0.5 + 0.5) * scale + offset_y << " " << vertex.position.z * scale + offset_z << std::endl;
				}
			}
			file << std::endl;

			// output texture coordinates
			file << "# List of texture coordinates" << std::endl;
			for (auto& face : faces) {
				if (face.texture_enabled) {
					for (auto& vertex : face.vertices) {
						file << "vt " << vertex.tex_coord.x << " " << vertex.tex_coord.y << std::endl;
					}
				}
			}
			file << std::endl;

			// output normal vectors
			file << "# List of vertex normals" << std::endl;
			for (auto& face : faces) {
				for (auto& vertex : face.vertices) {
					file << "vn " << vertex.normal.x << " " << vertex.normal.y << " " << vertex.normal.z << std::endl;
				}
			}
			file << std::endl;

			// output face information
			for (auto& face : faces) {
				file << "f";
				for (int k = 0; k < face.vertices.size(); k++) {
					file << " " << positionId;
					file << "/";
					file << "/" << positionId++;
				}
				file << std::endl;
			}

			file.close();
		}

		void OBJWriter::writeBuilding(std::shared_ptr<BuildingLayer> building, double scale, const cv::Point3f& color, const std::string& facade_texture, const std::string& roof_texture, std::vector<Face>& faces) {
			double roof_tile_size = 20.0 / scale;

			for (auto& footprint : building->footprints) {
				if (footprint.contour.size() == 0) continue;

				std::vector<std::vector<cv::Point2f>> polygons;

				cv::Rect bbox = util::boundingBox(footprint.contour.points);

				polygons = tessellate(footprint.contour, footprint.holes);
				for (int i = 0; i < polygons.size(); i++) {
					util::transform(polygons[i], footprint.mat);
				}

				for (auto polygon : polygons) {
					util::clockwise(polygon);

					std::vector<int> bottom_face(polygon.size());
					std::vector<int> top_face(polygon.size());

					std::vector<Vertex> bottom_vertices(polygon.size());
					std::vector<Vertex> top_vertices(polygon.size());
					for (int i = 0; i < polygon.size(); i++) {
						cv::Point3f pt_bottom(polygon[i].x, polygon[i].y, building->bottom_height);
						cv::Point3f pt_top(polygon[i].x, polygon[i].y, building->top_height);

						// calculate the texture coordinates
						cv::Point2f tex_coord((polygon[i].x - bbox.x) / roof_tile_size, (polygon[i].y - bbox.y) / roof_tile_size);

						top_vertices[i] = Vertex(pt_top, tex_coord, cv::Point3f(0, 0, 1));
						bottom_vertices[i] = Vertex(pt_bottom, cv::Point3f(0, 0, -1));
					}

					std::reverse(top_vertices.begin(), top_vertices.end());

					faces.push_back(Face(top_vertices, roof_texture));
					faces.push_back(Face(bottom_vertices, color));
				}
			}

			double height = building->top_height - building->bottom_height;
			double floor_tile_width = 2.4 / scale;
			double floor_tile_height = 3.0 / scale;

			// side faces
			for (auto& footprint : building->footprints) {
				if (footprint.contour.size() == 0) continue;

				util::Ring polygon = footprint.contour.getActualPoints();
				polygon.counterClockwise();
								
				// At first, find the first point that has angle close to 90 degrees.
				int start_index = 0;
				for (int i = 0; i < polygon.size(); i++) {
					int prev = (i + polygon.size() - 1) % polygon.size();
					int next = (i + 1) % polygon.size();

					if (dotProductBetweenThreePoints(polygon[prev], polygon[i], polygon[next]) < 0.8) {
						start_index = i;
						break;
					}
				}

				std::vector<cv::Point2f> coords;
				for (int i = 0; i <= polygon.size(); i++) {
					int prev = (start_index + i - 1 + polygon.size()) % polygon.size();
					int cur = (start_index + i) % polygon.size();
					int next = (start_index + i + 1) % polygon.size();

					if (dotProductBetweenThreePoints(polygon[prev], polygon[cur], polygon[next]) < 0.8) {
						// create a face
						if (coords.size() > 0) {
							coords.push_back(polygon[cur]);
							createFace(coords, building->bottom_height, height, floor_tile_width, floor_tile_height, color, facade_texture, faces);
						}
						coords.clear();
					}

					coords.push_back(polygon[cur]);
				}

				// create a face
				if (coords.size() >= 2) {
					createFace(coords, building->bottom_height, height, floor_tile_width, floor_tile_height, color, facade_texture, faces);
				}

				// side faces of holes
				for (auto& bh : footprint.holes) {
					if (bh.size() == 0) continue;

					util::Ring hole = bh.getActualPoints();
					hole.clockwise();

					// At first, find the first point that has angle close to 90 degrees.
					int start_index = 0;
					for (int i = 0; i < hole.size(); i++) {
						int prev = (i + hole.size() - 1) % hole.size();
						int next = (i + 1) % hole.size();

						if (dotProductBetweenThreePoints(hole[prev], hole[i], hole[next]) < 0.8) {
							start_index = i;
							break;
						}
					}

					std::vector<cv::Point2f> coords;
					for (int i = 0; i <= hole.size(); i++) {
						int prev = (start_index + i - 1 + hole.size()) % hole.size();
						int cur = (start_index + i) % hole.size();
						int next = (start_index + i + 1) % hole.size();

						if (dotProductBetweenThreePoints(hole[prev], hole[cur], hole[next]) < 0.8) {
							// create a face
							if (coords.size() > 0) {
								coords.push_back(hole[cur]);
								createFace(coords, building->bottom_height, height, floor_tile_width, floor_tile_height, color, facade_texture, faces);
							}
							coords.clear();
						}

						coords.push_back(hole[cur]);
					}

					// create a face
					if (coords.size() >= 2) {
						createFace(coords, building->bottom_height, height, floor_tile_width, floor_tile_height, color, facade_texture, faces);
					}
				}
			}	

			for (auto child : building->children) {
				writeBuilding(child, scale, color, facade_texture, roof_texture, faces);
			}
		}

		void OBJWriter::createFace(const std::vector<cv::Point2f>& coords, double z, double h, float floor_tile_width, float floor_tile_height, const cv::Point3f& color, const std::string& facade_texture, std::vector<Face>& faces) {
			double length = getLength(coords);
			int repetition = length / floor_tile_width;
			double actual_floor_tile_width = 0;
			if (repetition > 0) actual_floor_tile_width = length / repetition;

			double tex_x = 0;
			for (int i = 1; i < coords.size(); i++) {
				cv::Point3f p1(coords[i - 1].x, coords[i - 1].y, z);
				cv::Point3f p2(coords[i].x, coords[i].y, z);
				cv::Point3f p3(coords[i].x, coords[i].y, z + h);
				cv::Point3f p4(coords[i - 1].x, coords[i - 1].y, z + h);

				cv::Point2f t1(tex_x, 0);
				double tex_dx = 0;
				if (repetition > 0) {
					tex_dx = util::length(coords[i] - coords[i - 1]) / actual_floor_tile_width;
				}
				int tex_coord_y = h / floor_tile_height;
				cv::Point2f t2(tex_x + tex_dx, 0);
				cv::Point2f t3(tex_x + tex_dx, tex_coord_y);
				cv::Point2f t4(tex_x, tex_coord_y);

				tex_x += tex_dx;

				cv::Point3f n = crossProduct(p2 - p1, p3 - p2);
				n /= util::length(n);

				if (repetition == 0 || tex_coord_y == 0) {
					// the facade is too small, so we use a simple color
					std::vector<Vertex> vertices(4);
					vertices[0] = Vertex(p1, t1, n);
					vertices[1] = Vertex(p2, t2, n);
					vertices[2] = Vertex(p3, t3, n);
					vertices[3] = Vertex(p4, t4, n);
					faces.push_back(Face(vertices, color));
				}
				else {
					// the facade is big enough for texture mapping
					std::vector<Vertex> vertices(4);
					vertices[0] = Vertex(p1, t1, n);
					vertices[1] = Vertex(p2, t2, n);
					vertices[2] = Vertex(p3, t3, n);
					vertices[3] = Vertex(p4, t4, n);
					faces.push_back(Face(vertices, facade_texture));
				}
			}
		}
		
		double OBJWriter::dotProductBetweenThreePoints(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
			cv::Point2f prev_vec = b - a;
			cv::Point2f next_vec = c - b;
			prev_vec /= length(prev_vec);
			next_vec /= length(next_vec);

			return prev_vec.x * next_vec.x + prev_vec.y * next_vec.y;
		}

		double OBJWriter::getLength(const std::vector<cv::Point2f>& points) {
			double ans = 0;
			for (int i = 1; i < points.size(); i++) {
				ans += length(points[i] - points[i - 1]);
			}
			return ans;
		}

		void OBJWriter::writeVoxelNode(std::shared_ptr<VoxelNode> voxel_node, double scale, const cv::Point3f& color, std::vector<Face>& faces) {
			double height = 1;
			double z = voxel_node->height;

			Ring contour = voxel_node->contour.contour.getActualPoints();
			std::vector<Ring> holes(voxel_node->contour.holes.size());
			for (int i = 0; i < voxel_node->contour.holes.size(); i++) {
				holes[i] = voxel_node->contour.holes[i].getActualPoints();
			}

			std::vector<std::vector<cv::Point2f>> polygons;
			polygons = tessellate(contour, holes);
				
			for (auto polygon : polygons) {
				util::clockwise(polygon);

				std::vector<int> bottom_face(polygon.size());
				std::vector<int> top_face(polygon.size());

				std::vector<Vertex> bottom_vertices(polygon.size());
				std::vector<Vertex> top_vertices(polygon.size());
				for (int i = 0; i < polygon.size(); i++) {
					cv::Point3f pt_bottom(polygon[i].x, polygon[i].y, z);
					cv::Point3f pt_top(polygon[i].x, polygon[i].y, z + 1);

					top_vertices[i] = Vertex(pt_top, cv::Point3f(0, 0, 1));
					bottom_vertices[i] = Vertex(pt_bottom, cv::Point3f(0, 0, -1));
				}

				std::reverse(top_vertices.begin(), top_vertices.end());
				faces.push_back(Face(top_vertices, color));
				faces.push_back(Face(bottom_vertices, color));
			}

			// side faces
			util::Ring polygon = voxel_node->contour.contour.getActualPoints();
			polygon.counterClockwise();

			for (int i = 0; i < polygon.size(); i++) {
				int next = (i + 1) % polygon.size();

				cv::Point3f p1(polygon[i].x, polygon[i].y, z);
				cv::Point3f p2(polygon[next].x, polygon[next].y, z);
				cv::Point3f p3(polygon[next].x, polygon[next].y, z + 1);
				cv::Point3f p4(polygon[i].x, polygon[i].y, z + 1);

				cv::Point3f n = crossProduct(p2 - p1, p3 - p2);
				n /= util::length(n);

				std::vector<Vertex> vertices(4);
				vertices[0] = Vertex(p1, n);
				vertices[1] = Vertex(p2, n);
				vertices[2] = Vertex(p3, n);
				vertices[3] = Vertex(p4, n);
				faces.push_back(Face(vertices, color));
			}
					
			// side faces of holes
			for (auto& bh : voxel_node->contour.holes) {
				util::Ring hole = bh.getActualPoints();
				hole.clockwise();

				for (int i = 0; i < hole.size(); i++) {
					int next = (i + 1) % hole.size();

					cv::Point3f p1(hole[i].x, hole[i].y, z);
					cv::Point3f p2(hole[next].x, hole[next].y, z);
					cv::Point3f p3(hole[next].x, hole[next].y, z + 1);
					cv::Point3f p4(hole[i].x, hole[i].y, z + 1);

					cv::Point3f n = crossProduct(p2 - p1, p3 - p2);
					n /= util::length(n);

					std::vector<Vertex> vertices(4);
					vertices[0] = Vertex(p1, n);
					vertices[1] = Vertex(p2, n);
					vertices[2] = Vertex(p3, n);
					vertices[3] = Vertex(p4, n);
					faces.push_back(Face(vertices, color));
				}
			}
		}

	}

}
