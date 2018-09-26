#pragma once

#include <glew.h>
#include "Shader.h"
#include <QGLWidget>
#include <QMouseEvent>
#include "Camera.h"
#include "RenderManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "util/BuildingLayer.h"
#include "GLUtils.h"
#include "util/DisjointVoxelData.h"
#include "regularizer/Config.h"

class MainWindow;

class GLWidget3D : public QGLWidget {
	Q_OBJECT

public:
	enum SHOW_MODE { SHOW_INPUT = 0, SHOW_ALL, SHOW_DP, SHOW_RIGHTANGLE, SHOW_CURVE, SHOW_EFFICIENT_RANSAC };
	enum COLOR_MODE { COLOR = 0, TEXTURE };

public:
	MainWindow* mainWin;

	// camera
	Camera camera;
	glm::vec3 light_dir;
	glm::mat4 light_mvpMatrix;
	glm::vec3 spot_light_pos;

	// input voxel data
	cv::Point3i vdb_size;
	std::vector<util::VoxelBuilding> voxel_buildings;
	std::vector<std::shared_ptr<util::BuildingLayer>> buildings;
	glm::dvec3 offset;
	double scale;
	double min_hole_ratio;
	double min_height;

	// rendering engine
	RenderManager renderManager;
	int color_mode;
	int show_mode;
	
	// key status
	bool shiftPressed;
	bool ctrlPressed;

	bool first_paint;

public:
	GLWidget3D(MainWindow *parent = 0);

	void drawScene();
	void render();
	void loadVoxelData(const QString& filename);
	void saveOBJ(const QString& filename);
	void saveTopFace(const QString& filename);
	void savePLY(const QString& filename);
	void saveImage(const QString& filename);
	void showInputVoxel();
	void simplifyByAll(double alpha);
	void simplifyByDP(double epsilon, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang);
	void simplifyByRightAngle(int resolution, bool optimization, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang);
	void simplifyByCurve(double epsilon, double curve_threshold, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang);
	void simplifyByCurveRightAngle(double epsilon, double curve_threshold, double angle_threshold, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang);
	void simplifyByEfficientRansac(double curve_num_iterations, double curve_min_points, double curve_max_error_ratio_to_radius, double curve_cluster_epsilon, double curve_min_angle, double curve_min_radius, double curve_max_radius, double line_num_iterations, double line_min_points, double line_max_error, double line_cluster_epsilon, double line_min_length, double line_angle_threshold, double contour_max_error, double contour_angle_threshold, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, QString config_filename);
	void update3DGeometry();
	void update3DGeometry(const std::vector<util::VoxelBuilding>& voxel_buildings);
	void update3DGeometry(const std::shared_ptr<util::VoxelNode>& voxel_node, glm::vec4& color, std::vector<Vertex>& vertices);
	void update3DGeometryWithoutRoof(const std::vector<std::shared_ptr<util::BuildingLayer>>& buildings);
	void update3DGeometryWithoutRoof(std::shared_ptr<util::BuildingLayer> building, glm::vec4& color, const QString& facade_texture, const QString& roof_texture, QMap<QString, std::vector<Vertex>>& vertices);
	//void update3DGeometryWithRoof(const std::vector<std::shared_ptr<util::BuildingLayer>>& buildings);
	//void update3DGeometryWithRoof(std::shared_ptr<util::BuildingLayer> building, glm::vec4& color, const QString& facade_texture, const QString& roof_texture, QMap<QString, std::vector<Vertex>>& vertices);
	void createFace(const std::vector<glm::dvec2>& coords, double h, float floor_tile_width, float floor_tile_height, const glm::mat4& mat, glm::vec4& color, const QString& facade_texture, QMap<QString, std::vector<Vertex>>& vertices);
	double dotProductBetweenThreePoints(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);
	double getLength(const std::vector<glm::dvec2>& points);
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintEvent(QPaintEvent *event);
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void wheelEvent(QWheelEvent* e);
};
