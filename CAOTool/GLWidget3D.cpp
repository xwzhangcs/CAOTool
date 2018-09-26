#include "GLWidget3D.h"
#include "MainWindow.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <map>
#include <QDir>
#include <QTextStream>
#include <QDate>
#include <iostream>
#include <QProcess>
#include "GLUtils.h"
#include "util/ContourUtils.h"
#include "simp/BuildingSimplification.h"
#include "util/DisjointVoxelData.h"
#include "util/OBJWriter.h"
#include "util/PlyWriter.h"
#include "util/TopFaceWriter.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

GLWidget3D::GLWidget3D(MainWindow *parent) : QGLWidget(QGLFormat(QGL::SampleBuffers)) {
	this->mainWin = parent;
	ctrlPressed = false;
	shiftPressed = false;

	first_paint = true;

	// This is necessary to prevent the screen overdrawn by OpenGL
	setAutoFillBackground(false);

	// light direction for shadow mapping
	light_dir = glm::normalize(glm::vec3(-1, 1, -3));

	// model/view/projection matrices for shadow mapping
	glm::mat4 light_pMatrix = glm::ortho<float>(-1800, 1800, -1800, 1800, 0.1, 3600);
	glm::mat4 light_mvMatrix = glm::lookAt(-light_dir * 300.0f, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	light_mvpMatrix = light_pMatrix * light_mvMatrix;

	// spot light
	spot_light_pos = glm::vec3(20, 25, 30);

	offset = glm::dvec3(0, 0, 0);
	scale = 0.25;
	min_hole_ratio = 0.02;
	min_height = 1.5;

	color_mode = COLOR;
	show_mode = SHOW_INPUT;
}

/**
* Draw the scene.
*/
void GLWidget3D::drawScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(true);

	renderManager.renderAll();
}

void GLWidget3D::render() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PASS 1: Render to texture
	glUseProgram(renderManager.programs["pass1"]);

	glBindFramebuffer(GL_FRAMEBUFFER, renderManager.fragDataFB);
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderManager.fragDataTex[0], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, renderManager.fragDataTex[1], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, renderManager.fragDataTex[2], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, renderManager.fragDataTex[3], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderManager.fragDepthTex, 0);

	// Set the list of draw buffers.
	GLenum DrawBuffers[4] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
	glDrawBuffers(4, DrawBuffers); // "3" is the size of DrawBuffers
	// Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		printf("+ERROR: GL_FRAMEBUFFER_COMPLETE false\n");
		exit(0);
	}

	glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["pass1"], "mvpMatrix"), 1, false, &camera.mvpMatrix[0][0]);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "lightDir"), light_dir.x, light_dir.y, light_dir.z);
	glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["pass1"], "light_mvpMatrix"), 1, false, &light_mvpMatrix[0][0]);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "spotLightPos"), spot_light_pos.x, spot_light_pos.y, spot_light_pos.z);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "cameraPos"), camera.pos.x, camera.pos.y, camera.pos.z);

	glUniform1i(glGetUniformLocation(renderManager.programs["pass1"], "shadowMap"), 6);
	glActiveTexture(GL_TEXTURE6);
	glBindTexture(GL_TEXTURE_2D, renderManager.shadow.textureDepth);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	drawScene();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PASS 2: Create AO
	if (renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
		glUseProgram(renderManager.programs["ssao"]);
		glBindFramebuffer(GL_FRAMEBUFFER, renderManager.fragDataFB_AO);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderManager.fragAOTex, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderManager.fragDepthTex_AO, 0);
		GLenum DrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Always check that our framebuffer is ok
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			printf("++ERROR: GL_FRAMEBUFFER_COMPLETE false\n");
			exit(0);
		}

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["ssao"], "pixelSize"), 2.0f / this->width(), 2.0f / this->height());

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex0"), 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex1"), 2);
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "noiseTex"), 7);
		glActiveTexture(GL_TEXTURE7);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragNoiseTex);

		{
			glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["ssao"], "mvpMatrix"), 1, false, &camera.mvpMatrix[0][0]);
			glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["ssao"], "pMatrix"), 1, false, &camera.pMatrix[0][0]);
		}

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "uKernelSize"), renderManager.uKernelSize);
		glUniform3fv(glGetUniformLocation(renderManager.programs["ssao"], "uKernelOffsets"), renderManager.uKernelOffsets.size(), (const GLfloat*)renderManager.uKernelOffsets.data());

		glUniform1f(glGetUniformLocation(renderManager.programs["ssao"], "uPower"), renderManager.uPower);
		glUniform1f(glGetUniformLocation(renderManager.programs["ssao"], "uRadius"), renderManager.uRadius);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}
	else if (renderManager.renderingMode == RenderManager::RENDERING_MODE_LINE || renderManager.renderingMode == RenderManager::RENDERING_MODE_HATCHING) {
		glUseProgram(renderManager.programs["line"]);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["line"], "pixelSize"), 1.0f / this->width(), 1.0f / this->height());
		glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["line"], "pMatrix"), 1, false, &camera.pMatrix[0][0]);
		if (renderManager.renderingMode == RenderManager::RENDERING_MODE_HATCHING) {
			glUniform1i(glGetUniformLocation(renderManager.programs["line"], "useHatching"), 1);
		}
		else {
			glUniform1i(glGetUniformLocation(renderManager.programs["line"], "useHatching"), 0);
		}

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex0"), 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex1"), 2);
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex3"), 4);
		glActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[3]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "hatchingTexture"), 5);
		glActiveTexture(GL_TEXTURE5);
		glBindTexture(GL_TEXTURE_3D, renderManager.hatchingTextures);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}
	else if (renderManager.renderingMode == RenderManager::RENDERING_MODE_CONTOUR) {
		glUseProgram(renderManager.programs["contour"]);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["contour"], "pixelSize"), 1.0f / this->width(), 1.0f / this->height());

		glUniform1i(glGetUniformLocation(renderManager.programs["contour"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Blur

	if (renderManager.renderingMode == RenderManager::RENDERING_MODE_BASIC || renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUseProgram(renderManager.programs["blur"]);
		glUniform2f(glGetUniformLocation(renderManager.programs["blur"], "pixelSize"), 2.0f / this->width(), 2.0f / this->height());
		//printf("pixelSize loc %d\n", glGetUniformLocation(vboRenderManager.programs["blur"], "pixelSize"));

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex0"), 1);//COLOR
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex1"), 2);//NORMAL
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		/*glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);*/

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex3"), 4);//AO
		glActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragAOTex);

		if (renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
			glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "ssao_used"), 1); // ssao used
		}
		else {
			glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "ssao_used"), 0); // no ssao
		}

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);

	}

	// REMOVE
	glActiveTexture(GL_TEXTURE0);
}

void GLWidget3D::loadVoxelData(const QString& filename) {
	// get directory
	QDir dir = QFileInfo(filename).absoluteDir();

	// scan all the files in the directory to get a voxel data
	QStringList files = dir.entryList(QDir::NoDotAndDotDot | QDir::Files, QDir::DirsFirst);
	std::vector<cv::Mat_<uchar>> voxel_data(files.size());
	for (int i = 1; i < files.size(); i++) {
		voxel_data[i - 1] = cv::imread((dir.absolutePath() + "/" + files[i]).toUtf8().constData(), cv::IMREAD_GRAYSCALE);
	}
	vdb_size = cv::Point3i(voxel_data[0].cols, voxel_data[0].rows, voxel_data.size());

	voxel_buildings = util::DisjointVoxelData::disjoint(voxel_data);

	show_mode = SHOW_INPUT;
	update3DGeometry();
}

void GLWidget3D::saveOBJ(const QString& filename) {
	std::cout << "offset.x is " << offset.x << std::endl;
	std::cout << "offset.y is " << offset.y << std::endl;
	std::cout << "offset.z is " << offset.z << std::endl;
	if (show_mode == SHOW_INPUT) {
		util::obj::OBJWriter::writeVoxels(filename.toUtf8().constData(), vdb_size.x, vdb_size.y, offset.x, offset.y, offset.z, scale, voxel_buildings);
	}
	else {
		util::obj::OBJWriter::write(filename.toUtf8().constData(), vdb_size.x, vdb_size.y, offset.x, offset.y, offset.z, scale, buildings);
	}
}

void GLWidget3D::saveTopFace(const QString& filename) {
	util::topface::TopFaceWriter::write(filename.toUtf8().constData(), vdb_size.x, vdb_size.y, offset.x, offset.y, offset.z, scale, buildings);
}

void GLWidget3D::savePLY(const QString& filename) {
	util::ply::PlyWriter::write(filename.toUtf8().constData(), vdb_size.x, vdb_size.y, offset.x, offset.y, offset.z, scale, buildings);
}

void GLWidget3D::saveImage(const QString& filename) {
	QImage image = grabFrameBuffer();
	image.save(filename);
}

void GLWidget3D::showInputVoxel() {
	show_mode = SHOW_INPUT;
	update3DGeometry(voxel_buildings);
}

void GLWidget3D::simplifyByAll(double alpha) {
	// determine the layering threshold based on the weight ratio
	float threshold = 0.01;
	if (alpha < 0.1) threshold = 0.1;
	else if (alpha < 0.2) threshold = 0.3;
	else if (alpha < 0.3) threshold = 0.4;
	else if (alpha < 0.4) threshold = 0.5;
	else if (alpha < 0.5) threshold = 0.6;
	else if (alpha < 0.7) threshold = 0.6;
	else if (alpha < 0.9) threshold = 0.7;
	else threshold = 0.99;

	buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, simp::BuildingSimplification::ALG_ALL, false, min_height / scale, alpha, threshold, 0, 0, 0, 0, min_hole_ratio);

	show_mode = SHOW_ALL;
	update3DGeometry();
}

/**
* Simplify all the buildings by curve++ method.
*
* @param epsilon				The epsilon for DP method
* @param layering_threshold		layering threshold
* @param snapping_threshold		Threshold for snapping
* @param orientation			principle orientation of the building in radian
* @param min_contour_area		Minimum area of the contour [m^2].
* @param allow_triangle_contour	True if a triangle is allowed as a simplified contour shape
*/
void GLWidget3D::simplifyByDP(double epsilon, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang) {
	std::map<int, std::vector<double>> algorithms;
	algorithms[simp::BuildingSimplification::ALG_DP] = { epsilon };
	buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, algorithms, false, min_height / scale, 0.5, layering_threshold, snapping_threshold / scale, orientation, min_contour_area / scale / scale, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio);

	show_mode = SHOW_DP;
	update3DGeometry();
}

/**
* Simplify all the buildings by curve++ method.
*
* @param epsilon				The resolution
* @param layering_threshold		layering threshold
* @param snapping_threshold		Threshold for snapping
* @param orientation			principle orientation of the building in radian
* @param min_contour_area		Minimum area of the contour [m^2].
* @param allow_triangle_contour	True if a triangle is allowed as a simplified contour shape
*/
void GLWidget3D::simplifyByRightAngle(int resolution, bool optimization, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang) {
	std::map<int, std::vector<double>> algorithms;
	algorithms[simp::BuildingSimplification::ALG_RIGHTANGLE] = { (double)resolution, optimization ? 1.0 : 0.0 };
	buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, algorithms, false, min_height / scale, 0.5, layering_threshold, snapping_threshold / scale, orientation, min_contour_area / scale / scale, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio);

	show_mode = SHOW_RIGHTANGLE;
	update3DGeometry();
}

/**
* Simplify all the buildings by curve++ method.
*
* @param epsilon				The epsilon for DP method
* @param curve_threshold		curve threshold
* @param layering_threshold		layering threshold
* @param snapping_threshold		Threshold for snapping
* @param orientation			principle orientation of the building in radian
* @param min_contour_area		Minimum area of the contour [m^2].
* @param allow_triangle_contour	True if a triangle is allowed as a simplified contour shape
*/
void GLWidget3D::simplifyByCurve(double epsilon, double curve_threshold, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang) {
	std::map<int, std::vector<double>> algorithms;
	algorithms[simp::BuildingSimplification::ALG_CURVE] = { epsilon, curve_threshold };
	buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, algorithms, false, min_height / scale, 0.5, layering_threshold, snapping_threshold / scale, orientation, min_contour_area / scale / scale, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio);

	show_mode = SHOW_CURVE;
	update3DGeometry();
}

/**
 * Simplify all the buildings by curve++ method.
 * 
 * @param epsilon				The epsilon for DP method
 * @param curve_threshold		curve threshold
 * @param angle_threshold		angle threshold in radian
 * @param layering_threshold	layering threshold
 * @param snapping_threshold	Threshold for snapping
 * @param orientation			principle orientation of the building in radian
 * @param min_contour_area		Minimum area of the contour [m^2].
 * @param allow_triangle_contour	True if a triangle is allowed as a simplified contour shape
 */
void GLWidget3D::simplifyByCurveRightAngle(double epsilon, double curve_threshold, double angle_threshold, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang) {
	std::map<int, std::vector<double>> algorithms;
	algorithms[simp::BuildingSimplification::ALG_CURVE_RIGHTANGLE] = { epsilon, curve_threshold, angle_threshold };
	buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, algorithms, false, min_height / scale, 0.5, layering_threshold, snapping_threshold / scale, orientation, min_contour_area / scale / scale, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio);

	show_mode = SHOW_CURVE;
	update3DGeometry();
}

/**
* Simplify all the buildings by efficientRansac method.
*
* @param curve_num_iterations				The iterations for curve detection
* @param curve_min_points					minimum number of points on the curve
* @param curve_max_error_ratio_to_radius	max error for supporting points on the curve
* @param curve_cluster_epsilon				the cluster to choose curve
* @param curve_min_angle					minimum angle for the curve
* @param curve_min_radius					minimum radius for the curve
* @param curve_max_radius					maximum radius for the curve
* @param line_num_iterations				The iterations for line detection
* @param line_min_points					minimum number of points on the line segment
* @param line_max_error						max error for supporting points on the line
* @param line_cluster_epsilon				the cluster to choose line
* @param line_min_length					minimum length for the line
* @param line_angle_threshold				angle threshold for the line
* @param allow_triangle_contour	True if a triangle is allowed as a simplified contour shape
*/
void GLWidget3D::simplifyByEfficientRansac(double curve_num_iterations, double curve_min_points, double curve_max_error_ratio_to_radius, double curve_cluster_epsilon, double curve_min_angle, double curve_min_radius, double curve_max_radius, double line_num_iterations, double line_min_points, double line_max_error, double line_cluster_epsilon, double line_min_length, double line_angle_threshold, double contour_max_error, double contour_angle_threshold, double layering_threshold, double snapping_threshold, double orientation, double min_contour_area, float max_obb_ratio, bool allow_triangle_contour, bool allow_overhang, QString config_filename){
	std::map<int, std::vector<double>> algorithms;
	std::vector<regularizer::Config> regularizer_configs;
	algorithms[simp::BuildingSimplification::ALG_EFFICIENT_RANSAC] = { curve_num_iterations, curve_min_points, curve_max_error_ratio_to_radius, curve_cluster_epsilon, curve_min_angle, curve_min_radius, curve_max_radius, line_num_iterations, line_min_points, line_max_error, line_cluster_epsilon, line_min_length, line_angle_threshold, contour_max_error, contour_angle_threshold };
	if (!config_filename.isEmpty()){
		QFile file(config_filename);
		if (!file.open(QIODevice::ReadOnly)) {
			std::cerr << "File was not readable: " << std::endl;
		}
		else{
			QTextStream in(&file);
			rapidjson::Document doc;
			doc.Parse(in.readAll().toUtf8().constData());
			int num_runs = doc["num_runs"].GetInt();
			std::cout << "num_runs is " << num_runs << std::endl;
			regularizer_configs.resize(num_runs);
			for (int i = 0; i < num_runs; i++){
				bool bUseIntra = false;
				float intraWeight = 0.0f;
				bool bUseInter = false;
				float interWeight = 0.0f;
				bool bUseRaOpt = false;
				float angle_threshold_RA = 0.0f;
				float raWeight = 0.0f;
				bool bUseParallelOpt = false;
				float angle_threshold_parallel = 0.0f;
				float parallelWeight = 0.0f;
				bool bUseSymmetryLineOpt = false;
				float symmetryIouThreshold = 0.0f;
				float symmetryWeight = 0.0f;
				bool bUseAccuracyOpt = false;
				float accuracyWeight = 0.0f;
				bool bUsePointSnapOpt = false;
				float pointDisThreshold = 0.0f;
				float pointWeight = 0.0f;
				bool bUseSegSnapOpt = false;
				float segDisThreshold = 0.0f;
				float segAngleThreshold = 0.0f;
				float segWeight = 0.0f;

				// get parameters for each run
				QString run_name = "Opt" + QString::number(i + 1);
				rapidjson::Value& algs_run = doc[run_name.toUtf8().constData()];
				
				bUseIntra = algs_run["UseIntra"].GetBool();
				bUseInter = algs_run["UseInter"].GetBool();
				intraWeight = algs_run["Intra_Weight"].GetFloat();
				interWeight = algs_run["Inter_Weight"].GetFloat();
				if (bUseIntra){
					rapidjson::Value& algs = algs_run["IntraOpt"];
					//ra
					rapidjson::Value& algs_ra = algs["RA"];
					bUseRaOpt = algs_ra["UseOpt"].GetBool();
					angle_threshold_RA = algs_ra["AngleThreshold"].GetFloat();
					raWeight = algs_ra["Weight"].GetFloat();
					//symmetry
					rapidjson::Value& algs_symmetry = algs["Symmetry"]; 
					bUseSymmetryLineOpt = algs_symmetry["UseOpt"].GetBool();
					symmetryIouThreshold = algs_symmetry["IouThreshold"].GetFloat();
					symmetryWeight = algs_symmetry["Weight"].GetFloat();
					// parallel
					rapidjson::Value& algs_parallel = algs["Parallel"];
					bUseParallelOpt = algs_parallel["UseOpt"].GetBool();
					angle_threshold_parallel = algs_parallel["AngleThreshold"].GetFloat();
					parallelWeight = algs_parallel["Weight"].GetFloat();
					// accuracy
					rapidjson::Value& algs_accuracy = algs["Accuracy"];
					bUseAccuracyOpt = algs_accuracy["UseOpt"].GetBool();
					accuracyWeight = algs_accuracy["Weight"].GetFloat();
				}
				if (bUseInter){
					rapidjson::Value& algs = algs_run["InterOpt"];
					// point snap
					rapidjson::Value& algs_point = algs["PointSnap"];
					bUsePointSnapOpt = algs_point["UseOpt"].GetBool();
					pointDisThreshold = algs_point["DisThreshold"].GetFloat();
					pointWeight = algs_point["Weight"].GetFloat();
					// seg snap
					rapidjson::Value& algs_seg = algs["SegSnap"];
					bUseSegSnapOpt = algs_seg["UseOpt"].GetBool();
					segDisThreshold = algs_seg["DisThreshold"].GetFloat();
					segAngleThreshold = algs_seg["AngleThreshold"].GetFloat();
					segWeight = algs_seg["Weight"].GetFloat();
				}
				//check weights
				if (bUseInter || bUseIntra){
					float weight = 0.0f;
					if (bUseInter)
						weight += interWeight;
					if (bUseIntra)
						weight += intraWeight;
					if (abs(weight - 1.0f) < 0.0001)
					{
						//
					}
					else{
						std::cout << "Please check intra and inter weight assignment!!!" << std::endl;
						return;
					}
				}
				if (bUseIntra){
					//check the sum weight equals 1
					float weight = 0.0f;
					if (bUseRaOpt)
						weight += raWeight;
					if (bUseParallelOpt)
						weight += parallelWeight;
					if (bUseSymmetryLineOpt)
						weight += symmetryWeight;
					if (bUseAccuracyOpt)
						weight += accuracyWeight;
					if (abs(weight - 1.0f) < 0.0001)
					{
						//
					}
					else{
						std::cout << "Please check intra weight assignment!!!" << std::endl;
						return;
					}
				}
				if (bUseInter){
					float weight = 0.0f;
					if (bUsePointSnapOpt)
						weight += pointWeight;
					if (bUseSegSnapOpt)
						weight += segWeight;
					if (abs(weight - 1.0f) < 0.0001)
					{
						//
					}
					else{
						std::cout << "Please check inter weight assignment!!!" << std::endl;
						return;
					}
				}
				regularizer::Config config(bUseIntra, intraWeight, bUseInter, interWeight, bUseRaOpt, angle_threshold_RA, raWeight, bUseParallelOpt, angle_threshold_parallel, parallelWeight, bUseSymmetryLineOpt, symmetryIouThreshold, symmetryWeight, bUseAccuracyOpt, accuracyWeight, bUsePointSnapOpt, pointDisThreshold, pointWeight, bUseSegSnapOpt, segDisThreshold, segAngleThreshold, segWeight);
				regularizer_configs[i] = config;
				/*{
					std::cout << "bUseRa " << config.bUseRaOpt << " ra angle is " << config.angle_threshold_RA << " ra weight is " << config.raWeight << std::endl;
					std::cout << "bUseParallel " << config.bUseParallelOpt << " Parallel angle is " << config.angle_threshold_parallel << " Parallel weight is " << config.parallelWeight << std::endl;
					std::cout << "bUseSymmetry " << config.bUseSymmetryLineOpt << " Symmetry weight is " << config.symmetryWeight << std::endl;
					std::cout << "bUseAccuracy " << config.bUseAccuracyOpt << " Accuracy weight is " << config.accuracyWeight << std::endl;
				}
				{
					std::cout << "bUsePoint " << config.bUsePointSnapOpt << " Point threshold is " << config.pointDisThreshold << " Point weight is " << config.pointWeight << std::endl;
					std::cout << "bUseSeg " << config.bUseSegSnapOpt << " seg angle is " << config.segAngleThreshold << " seg weight is " << config.segWeight << std::endl;
				}*/
			}
			file.close();
		}
	}
	buildings = simp::BuildingSimplification::simplifyBuildings(voxel_buildings, algorithms, false, min_height / scale, 0.5, layering_threshold, snapping_threshold / scale, orientation, min_contour_area / scale / scale, max_obb_ratio, allow_triangle_contour, allow_overhang, min_hole_ratio, regularizer_configs);

	show_mode = SHOW_EFFICIENT_RANSAC;
	update3DGeometry();
}

void GLWidget3D::update3DGeometry() {
	if (show_mode == SHOW_INPUT) {
		update3DGeometry(voxel_buildings);
	}
	else {
		update3DGeometryWithoutRoof(buildings);
	}
}

/**
* Update 3D geometry using the input building shapes.
*
* @param buildings		buildings
*/
void GLWidget3D::update3DGeometry(const std::vector<util::VoxelBuilding>& voxel_buildings) {
	renderManager.removeObjects();

	glm::vec4 color(0.7, 1, 0.7, 1);

	std::vector<Vertex> vertices;
	for (int i = 0; i < voxel_buildings.size(); i++) {
		for (int z = 0; z < voxel_buildings[i].node_stack.size(); z++) {
			for (auto voxel_node : voxel_buildings[i].node_stack[z]) {
				update3DGeometry(voxel_node, color, vertices);
			}
		}
	}
	renderManager.addObject("building", "", vertices, true);

	std::vector<Vertex> vertices2;
	glutils::drawBox(3000, 3000, 0.5, glm::vec4(0.9, 1, 0.9, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -0.5)), vertices2);
	renderManager.addObject("ground", "", vertices2, true);

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}

void GLWidget3D::update3DGeometry(const std::shared_ptr<util::VoxelNode>& voxel_node, glm::vec4& color, std::vector<Vertex>& vertices) {
	std::vector<glm::dvec2> footprint(voxel_node->contour.contour.size());
	for (int i = 0; i < voxel_node->contour.contour.size(); i++) {
		cv::Point2f pt = voxel_node->contour.contour.getActualPoint(i);
		footprint[i] = glm::dvec2(pt.x * scale, pt.y * scale);
	}
	std::vector<std::vector<glm::dvec2>> holes(voxel_node->contour.holes.size());
	for (int i = 0; i < voxel_node->contour.holes.size(); i++) {
		if (voxel_node->contour.holes[i].size() < 3) continue;
		holes[i].resize(voxel_node->contour.holes[i].size());
		for (int j = 0; j < voxel_node->contour.holes[i].size(); j++) {
			cv::Point2f pt = voxel_node->contour.holes[i].getActualPoint(j);
			holes[i][j] = glm::dvec2(pt.x * scale, pt.y * scale);
		}
	}

	// correct the orientation of polygon
	glutils::correct(footprint);
	for (int i = 0; i < holes.size(); i++) {
		glutils::correct(holes[i]);
	}

	glm::mat4 mat = glm::translate(glm::mat4(), glm::vec3(0, 0, voxel_node->height * scale));
	double h = scale;

	// top face
	glutils::drawConcavePolygon(footprint, holes, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices);

	// bottom face
	glutils::drawConcavePolygon(footprint, holes, color, mat, vertices, true);

	// side faces
	for (int i = 0; i < footprint.size(); i++) {
		int next = (i + 1) % footprint.size();

		glm::vec3 p1(mat * glm::vec4(footprint[i], 0, 1));
		glm::vec3 p2(mat * glm::vec4(footprint[next], 0, 1));
		glm::vec3 p3(mat * glm::vec4(footprint[next], h, 1));
		glm::vec3 p4(mat * glm::vec4(footprint[i], h, 1));

		glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
		n /= glm::length(n);

		vertices.push_back(Vertex(p1, n, color));
		vertices.push_back(Vertex(p2, n, color));
		vertices.push_back(Vertex(p3, n, color));

		vertices.push_back(Vertex(p1, n, color));
		vertices.push_back(Vertex(p3, n, color));
		vertices.push_back(Vertex(p4, n, color));
	}

	// side faces for the holes
	for (auto& hole : holes) {
		std::reverse(hole.begin(), hole.end());
		for (int j = 0; j < hole.size(); j++) {
			int next = (j + 1) % hole.size();

			glm::vec3 p1(mat * glm::vec4(hole[j], 0, 1));
			glm::vec3 p2(mat * glm::vec4(hole[next], 0, 1));
			glm::vec3 p3(mat * glm::vec4(hole[next], h, 1));
			glm::vec3 p4(mat * glm::vec4(hole[j], h, 1));

			glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
			n /= glm::length(n);

			vertices.push_back(Vertex(p1, n, color));
			vertices.push_back(Vertex(p2, n, color));
			vertices.push_back(Vertex(p3, n, color));

			vertices.push_back(Vertex(p1, n, color));
			vertices.push_back(Vertex(p3, n, color));
			vertices.push_back(Vertex(p4, n, color));
		}
	}
}

/**
 * Update 3D geometry using the simplified building shapes.
 * Use the generated primitive shapes to create the 3D mesh.
 *
 * @param buildings		buildings
 */
void GLWidget3D::update3DGeometryWithoutRoof(const std::vector<std::shared_ptr<util::BuildingLayer>>& buildings) {
	renderManager.removeObjects();

	glm::vec4 color(0.7, 1, 0.7, 1);

	QMap<QString, std::vector<Vertex>> vertices;
	for (int i = 0; i < buildings.size(); i++) {
		// randomly select a texture file
		int facade_texture_id = rand() % 13;
		if (color_mode == TEXTURE) {
			if (facade_texture_id == 0) {
				color = glm::vec4(0.741, 0.623, 0.490, 1.0);
			}
			else if (facade_texture_id == 1) {
				color = glm::vec4(0.792, 0.768, 0.721, 1.0);
			}
			else if (facade_texture_id == 2) {
				color = glm::vec4(0.772, 0.709, 0.647, 1.0);
			}
			else if (facade_texture_id == 3) {
				color = glm::vec4(0.674, 0.639, 0.572, 1.0);
			}
			else if (facade_texture_id == 4) {
				color = glm::vec4(0.776, 0.819, 0.843, 1.0);
			}
			else if (facade_texture_id == 5) {
				color = glm::vec4(0.882, 0.882, 0.874, 1.0);
			}
			else if (facade_texture_id == 6) {
				color = glm::vec4(0.811, 0.811, 0.803, 1.0);
			}
			else if (facade_texture_id == 7) {
				color = glm::vec4(0.850, 0.831, 0.815, 1.0);
			}
			else if (facade_texture_id == 8) {
				color = glm::vec4(0.811, 0.831, 0.843, 1.0);
			}
			else if (facade_texture_id == 9) {
				color = glm::vec4(0.784, 0.721, 0.623, 1.0);
			}
			else if (facade_texture_id == 10) {
				color = glm::vec4(0.494, 0.388, 0.321, 1.0);
			}
			else if (facade_texture_id == 11) {
				color = glm::vec4(0.756, 0.780, 0.772, 1.0);
			}
			else if (facade_texture_id == 12) {
				color = glm::vec4(0.764, 0.768, 0.788, 1.0);
			}
		}

		QString facade_texture = QString("textures/window_tile%1.png").arg(facade_texture_id);
		QString roof_texture = QString("textures/roof%1.png").arg(rand() % 4);
		update3DGeometryWithoutRoof(buildings[i], color, facade_texture, roof_texture, vertices);
	}
	for (auto it = vertices.begin(); it != vertices.end(); it++) {
		renderManager.addObject("building", it.key(), it.value(), true);
	}

	std::vector<Vertex> vertices2;
	glutils::drawBox(3000, 3000, 0.5, glm::vec4(0.9, 1, 0.9, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -0.5)), vertices2);
	renderManager.addObject("ground", "", vertices2, true);

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}

void GLWidget3D::update3DGeometryWithoutRoof(std::shared_ptr<util::BuildingLayer> building, glm::vec4& color, const QString& facade_texture, const QString& roof_texture, QMap<QString, std::vector<Vertex>>& vertices) {
	float floor_tile_width = 2.4f;
	float floor_tile_height = 3.0f;

	for (auto& bf : building->footprints) {
		if (bf.contour.size() == 0) continue;

		std::vector<std::vector<cv::Point2f>> polygons;

		polygons = util::tessellate(bf.contour, bf.holes);
		for (int i = 0; i < polygons.size(); i++) {
			util::transform(polygons[i], bf.mat);
		}

		glm::mat4 mat = glm::translate(glm::mat4(), glm::vec3(0, 0, building->bottom_height * scale));
		double height = (building->top_height - building->bottom_height) * scale;

		for (auto polygon : polygons) {
			util::clockwise(polygon);

			std::vector<glm::dvec2> pol(polygon.size());

			for (int i = 0; i < polygon.size(); i++) {
				pol[i] = glm::dvec2(polygon[i].x * scale, polygon[i].y * scale);
			}

			// top face
			glutils::drawPolygon(pol, color, glm::translate(glm::mat4(), glm::vec3(0, 0, building->top_height * scale)), vertices[""]);

			// bottom face
			glutils::drawPolygon(pol, color, mat, vertices[""], true);
		}

		util::Ring ring = bf.contour.getActualPoints();
		ring.counterClockwise();
		std::vector<glm::dvec2> pol(ring.size());
		for (int i = 0; i < ring.size(); i++) {
			pol[i] = glm::dvec2(ring[i].x * scale, ring[i].y * scale);
		}

		// side faces

		// At first, find the first point that has angle close to 90 degrees.
		int start_index = 0;
		for (int i = 0; i < pol.size(); i++) {
			int prev = (i + pol.size() - 1) % pol.size();
			int next = (i + 1) % pol.size();

			if (dotProductBetweenThreePoints(pol[prev], pol[i], pol[next]) < 0.8) {
				start_index = i;
				break;
			}
		}

		std::vector<glm::dvec2> coords;
		for (int i = 0; i <= pol.size(); i++) {
			int prev = (start_index + i - 1 + pol.size()) % pol.size();
			int cur = (start_index + i) % pol.size();
			int next = (start_index + i + 1) % pol.size();

			if (dotProductBetweenThreePoints(pol[prev], pol[cur], pol[next]) < 0.8) {
				// create a face
				if (coords.size() > 0) {
					coords.push_back(pol[cur]);
					createFace(coords, height, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
				}
				coords.clear();
			}

			coords.push_back(pol[cur]);
		}

		// create a face
		if (coords.size() >= 2) {
			createFace(coords, height, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
		}

		// side faces of holes
		for (auto& bh : bf.holes) {
			util::Ring ring = bh.getActualPoints();
			ring.clockwise();
			std::vector<glm::dvec2> pol(ring.size());
			for (int i = 0; i < ring.size(); i++) {
				pol[i] = glm::dvec2(ring[i].x * scale, ring[i].y * scale);
			}

			// side faces

			// At first, find the first point that has angle close to 90 degrees.
			int start_index = 0;
			for (int i = 0; i < pol.size(); i++) {
				int prev = (i + pol.size() - 1) % pol.size();
				int next = (i + 1) % pol.size();

				if (dotProductBetweenThreePoints(pol[prev], pol[i], pol[next]) < 0.8) {
					start_index = i;
					break;
				}
			}

			std::vector<glm::dvec2> coords;
			for (int i = 0; i <= pol.size(); i++) {
				int prev = (start_index + i - 1 + pol.size()) % pol.size();
				int cur = (start_index + i) % pol.size();
				int next = (start_index + i + 1) % pol.size();

				if (dotProductBetweenThreePoints(pol[prev], pol[cur], pol[next]) < 0.8) {
					// create a face
					if (coords.size() > 0) {
						coords.push_back(pol[cur]);
						createFace(coords, height, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
					}
					coords.clear();
				}

				coords.push_back(pol[cur]);
			}

			// create a face
			if (coords.size() >= 2) {
				createFace(coords, height, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
			}
		}
	}

	for (auto child : building->children) {
		update3DGeometryWithoutRoof(child, color, facade_texture, roof_texture, vertices);
	}
}

/**
 * Update 3D geometry using the simplified building shapes.
 * Use the contour polygon to generate a flat roof.
 *
 * @param buildings		buildings
 */
/*
void GLWidget3D::update3DGeometryWithRoof(const std::vector<std::shared_ptr<util::BuildingLayer>>& buildings) {
	renderManager.removeObjects();

	glm::vec4 color(0.7, 1, 0.7, 1);

	QMap<QString, std::vector<Vertex>> vertices;
	for (int i = 0; i < buildings.size(); i++) {
		// randomly select a texture file
		int facade_texture_id = rand() % 13;
		if (color_mode == TEXTURE) {
			if (facade_texture_id == 0) {
				color = glm::vec4(0.741, 0.623, 0.490, 1.0);
			}
			else if (facade_texture_id == 1) {
				color = glm::vec4(0.792, 0.768, 0.721, 1.0);
			}
			else if (facade_texture_id == 2) {
				color = glm::vec4(0.772, 0.709, 0.647, 1.0);
			}
			else if (facade_texture_id == 3) {
				color = glm::vec4(0.674, 0.639, 0.572, 1.0);
			}
			else if (facade_texture_id == 4) {
				color = glm::vec4(0.776, 0.819, 0.843, 1.0);
			}
			else if (facade_texture_id == 5) {
				color = glm::vec4(0.882, 0.882, 0.874, 1.0);
			}
			else if (facade_texture_id == 6) {
				color = glm::vec4(0.811, 0.811, 0.803, 1.0);
			}
			else if (facade_texture_id == 7) {
				color = glm::vec4(0.850, 0.831, 0.815, 1.0);
			}
			else if (facade_texture_id == 8) {
				color = glm::vec4(0.811, 0.831, 0.843, 1.0);
			}
			else if (facade_texture_id == 9) {
				color = glm::vec4(0.784, 0.721, 0.623, 1.0);
			}
			else if (facade_texture_id == 10) {
				color = glm::vec4(0.494, 0.388, 0.321, 1.0);
			}
			else if (facade_texture_id == 11) {
				color = glm::vec4(0.756, 0.780, 0.772, 1.0);
			}
			else if (facade_texture_id == 12) {
				color = glm::vec4(0.764, 0.768, 0.788, 1.0);
			}
		}

		QString facade_texture = QString("textures/window_tile%1.png").arg(facade_texture_id);
		QString roof_texture = QString("textures/roof%1.png").arg(rand() % 4);
		update3DGeometryWithRoof(buildings[i], color, facade_texture, roof_texture, vertices);
	}
	for (auto it = vertices.begin(); it != vertices.end(); it++) {
		renderManager.addObject("building", it.key(), it.value(), true);
	}

	std::vector<Vertex> vertices2;
	glutils::drawBox(1500, 1500, 0.5, glm::vec4(0.9, 1, 0.9, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -0.5)), vertices2);
	renderManager.addObject("ground", "", vertices2, true);

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}
*/

/*
void GLWidget3D::update3DGeometryWithRoof(std::shared_ptr<util::BuildingLayer> building, glm::vec4& color, const QString& facade_texture, const QString& roof_texture, QMap<QString, std::vector<Vertex>>& vertices) {
	if (building->footprint.contour.size() < 3) return;

	std::vector<glm::dvec2> footprint(building->footprint.contour.size());
	for (int j = 0; j < building->footprint.contour.size(); j++) {
		cv::Point2f pt = building->footprint.contour.getActualPoint(j);
		footprint[j] = glm::dvec2(pt.x, pt.y);
	}
	std::vector<std::vector<glm::dvec2>> holes(building->footprint.holes.size());
	for (int j = 0; j < building->footprint.holes.size(); j++) {
		if (building->footprint.holes[j].size() < 3) continue;
		holes[j].resize(building->footprint.holes[j].size());
		for (int k = 0; k < building->footprint.holes[j].size(); k++) {
			cv::Point2f pt = building->footprint.holes[j].getActualPoint(k);
			holes[j][k] = glm::dvec2(pt.x, pt.y);
		}
	}

	// correct the orientation of polygon
	glutils::correct(footprint);
	for (int i = 0; i < holes.size(); i++) {
		glutils::correct(holes[i]);
	}

	glm::mat4 mat = glm::translate(glm::mat4(), glm::vec3(0, 0, building->bottom_height));
	double h = building->top_height - building->bottom_height;

	// top face
	if (color_mode == COLOR) {
		glutils::drawConcavePolygon(footprint, holes, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices[""]);
	}
	else {
		glm::mat4 topface_mat = glm::translate(mat, glm::vec3(0, 0, h));

		// dilate the contour
		std::vector<glm::dvec2> dilated_footprint;
		glutils::offsetPolygon(footprint, 0.2, dilated_footprint);
		std::vector<std::vector<glm::dvec2>> dilated_holes(holes.size());
		for (int i = 0; i < dilated_holes.size(); i++) {
			glutils::offsetPolygon(holes[i], -0.2, dilated_holes[i]);
		}

		// bottom face of the roof
		glutils::drawConcavePolygon(dilated_footprint, dilated_holes, color, topface_mat, vertices[""], true);

		// side faces of the roof
		double roof_ledge_height = 0.2;
		for (int i = 0; i < dilated_footprint.size(); i++) {
			int next = (i + 1) % dilated_footprint.size();
			glm::vec3 p1(topface_mat * glm::vec4(dilated_footprint[i], 0, 1));
			glm::vec3 p2(topface_mat * glm::vec4(dilated_footprint[next], 0, 1));
			glm::vec3 p3(topface_mat * glm::vec4(dilated_footprint[next], roof_ledge_height, 1));
			glm::vec3 p4(topface_mat * glm::vec4(dilated_footprint[i], roof_ledge_height, 1));

			glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
			n /= glm::length(n);

			vertices[""].push_back(Vertex(p1, n, color));
			vertices[""].push_back(Vertex(p2, n, color));
			vertices[""].push_back(Vertex(p3, n, color));

			vertices[""].push_back(Vertex(p1, n, color));
			vertices[""].push_back(Vertex(p3, n, color));
			vertices[""].push_back(Vertex(p4, n, color));
		}

		// side faces for the holes of the roof
		for (int i = 0; i < dilated_holes.size(); i++) {
			reverse(dilated_holes[i].begin(), dilated_holes[i].end());
			for (int j = 0; j < dilated_holes[i].size(); j++) {
				int next = (j + 1) % dilated_holes[i].size();
				glm::vec3 p1(topface_mat * glm::vec4(dilated_holes[i][j], 0, 1));
				glm::vec3 p2(topface_mat * glm::vec4(dilated_holes[i][next], 0, 1));
				glm::vec3 p3(topface_mat * glm::vec4(dilated_holes[i][next], roof_ledge_height, 1));
				glm::vec3 p4(topface_mat * glm::vec4(dilated_holes[i][j], roof_ledge_height, 1));

				glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
				n /= glm::length(n);

				// the facade is too small, so we use a simple color
				vertices[""].push_back(Vertex(p1, n, color));
				vertices[""].push_back(Vertex(p2, n, color));
				vertices[""].push_back(Vertex(p3, n, color));

				vertices[""].push_back(Vertex(p1, n, color));
				vertices[""].push_back(Vertex(p3, n, color));
				vertices[""].push_back(Vertex(p4, n, color));
			}
		}

		// top of the roof
		std::vector<glm::dvec2> texCoords(dilated_footprint.size());
		std::vector<std::vector<glm::dvec2>> holes_texCoords(dilated_holes.size());
		glutils::BoundingBox bbox(dilated_footprint);
		for (int i = 0; i < dilated_footprint.size(); i++) {
			texCoords[i] = glm::vec2((dilated_footprint[i].x - bbox.minPt.x) / 10, (dilated_footprint[i].y - bbox.minPt.y) / 10);
		}
		for (int i = 0; i < dilated_holes.size(); i++) {
			holes_texCoords[i].resize(holes[i].size());
			for (int j = 0; j < holes[i].size(); j++) {
				holes_texCoords[i][j] = glm::vec2((dilated_holes[i][j].x - bbox.minPt.x) / 10, (dilated_holes[i][j].y - bbox.minPt.y) / 10);
			}
		}
		glutils::drawConcavePolygon(dilated_footprint, dilated_holes, glm::vec4(1, 1, 1, 1), texCoords, holes_texCoords, glm::translate(topface_mat, glm::vec3(0, 0, roof_ledge_height)), vertices[roof_texture]);
	}

	// bottom face
	glutils::drawConcavePolygon(footprint, holes, color, mat, vertices[""], true);


	// side faces
	float floor_tile_width = 3.0f;
	float floor_tile_height = 3.0f;

	// At first, find the first point that has angle close to 90 degrees.
	int start_index = 0;
	for (int i = 0; i < footprint.size(); i++) {
		int prev = (i + footprint.size() - 1) % footprint.size();
		int next = (i + 1) % footprint.size();

		if (dotProductBetweenThreePoints(footprint[prev], footprint[i], footprint[next]) < 0.8) {
			start_index = i;
			break;
		}
	}

	std::vector<glm::dvec2> coords;
	for (int i = 0; i <= footprint.size(); i++) {
		int prev = (start_index + i - 1 + footprint.size()) % footprint.size();
		int cur = (start_index + i) % footprint.size();
		int next = (start_index + i + 1) % footprint.size();

		if (dotProductBetweenThreePoints(footprint[prev], footprint[cur], footprint[next]) < 0.8) {
			// create a face
			if (coords.size() > 0) {
				coords.push_back(footprint[cur]);
				createFace(coords, h, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
			}
			coords.clear();
		}

		coords.push_back(footprint[cur]);
	}

	// create a face
	if (coords.size() >= 2) {
		createFace(coords, h, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
	}

	// side faces for the holes
	for (int i = 0; i < holes.size(); i++) {
		reverse(holes[i].begin(), holes[i].end());

		// At first, find the first point that has angle close to 90 degrees.
		int start_index = 0;
		for (int j = 0; j < holes[i].size(); j++) {
			int prev = (j + holes[i].size() - 1) % holes[i].size();
			int next = (j + 1) % holes[i].size();

			if (dotProductBetweenThreePoints(holes[i][prev], holes[i][i], holes[i][next]) < 0.8) {
				start_index = j;
				break;
			}
		}

		std::vector<glm::dvec2> coords;
		for (int j = 0; j <= holes[i].size(); j++) {
			int prev = (start_index + j - 1 + holes[i].size()) % holes[i].size();
			int cur = (start_index + j) % holes[i].size();
			int next = (start_index + j + 1) % holes[i].size();

			if (dotProductBetweenThreePoints(holes[i][prev], holes[i][cur], holes[i][next]) < 0.8) {
				// create a face
				if (coords.size() > 0) {
					coords.push_back(holes[i][cur]);
					createFace(coords, h, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
				}
				coords.clear();
			}

			coords.push_back(holes[i][cur]);
		}

		// create a face
		if (coords.size() >= 2) {
			createFace(coords, h, floor_tile_width, floor_tile_height, mat, color, facade_texture, vertices);
		}
	}

	if (building->child) {
		update3DGeometryWithRoof(building->child, color, facade_texture, roof_texture, vertices);
	}
}
*/

void GLWidget3D::createFace(const std::vector<glm::dvec2>& coords, double h, float floor_tile_width, float floor_tile_height, const glm::mat4& mat, glm::vec4& color, const QString& facade_texture, QMap<QString, std::vector<Vertex>>& vertices) {
	float length = getLength(coords);
	int repetition = length / floor_tile_width;
	float actual_floor_tile_width = 0;
	if (repetition > 0) actual_floor_tile_width = length / repetition;

	float tex_x = 0;
	for (int i = 1; i < coords.size(); i++) {
		glm::vec3 p1(mat * glm::vec4(coords[i - 1], 0, 1));
		glm::vec3 p2(mat * glm::vec4(coords[i], 0, 1));
		glm::vec3 p3(mat * glm::vec4(coords[i], h, 1));
		glm::vec3 p4(mat * glm::vec4(coords[i - 1], h, 1));

		glm::vec2 t1(tex_x, 0);
		float tex_dx = 0;
		if (repetition > 0) {
			tex_dx = glm::length(coords[i] - coords[i - 1]) / actual_floor_tile_width;
		}
		int tex_coord_y = h / floor_tile_height;
		glm::vec2 t2(tex_x + tex_dx, 0);
		glm::vec2 t3(tex_x + tex_dx, tex_coord_y);
		glm::vec2 t4(tex_x, tex_coord_y);

		tex_x += tex_dx;

		glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
		n /= glm::length(n);

		if (color_mode == COLOR || repetition == 0 || tex_coord_y == 0) {
			// the facade is too small, so we use a simple color
			vertices[""].push_back(Vertex(p1, n, color));
			vertices[""].push_back(Vertex(p2, n, color));
			vertices[""].push_back(Vertex(p3, n, color));

			vertices[""].push_back(Vertex(p1, n, color));
			vertices[""].push_back(Vertex(p3, n, color));
			vertices[""].push_back(Vertex(p4, n, color));
		}
		else {
			// the facade is big enough for texture mapping
			vertices[facade_texture].push_back(Vertex(p1, n, glm::vec4(1, 1, 1, 1), t1));
			vertices[facade_texture].push_back(Vertex(p2, n, glm::vec4(1, 1, 1, 1), t2));
			vertices[facade_texture].push_back(Vertex(p3, n, glm::vec4(1, 1, 1, 1), t3));

			vertices[facade_texture].push_back(Vertex(p1, n, glm::vec4(1, 1, 1, 1), t1));
			vertices[facade_texture].push_back(Vertex(p3, n, glm::vec4(1, 1, 1, 1), t3));
			vertices[facade_texture].push_back(Vertex(p4, n, glm::vec4(1, 1, 1, 1), t4));
		}		
	}
}

double GLWidget3D::dotProductBetweenThreePoints(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
	glm::dvec2 prev_vec = b - a;
	glm::dvec2 next_vec = c - b;
	prev_vec /= glm::length(prev_vec);
	next_vec /= glm::length(next_vec);
	return glm::dot(prev_vec, next_vec);
}

double GLWidget3D::getLength(const std::vector<glm::dvec2>& points) {
	double ans = 0;
	for (int i = 1; i < points.size(); i++) {
		ans += glm::length(points[i] - points[i - 1]);
	}
	return ans;
}

void GLWidget3D::keyPressEvent(QKeyEvent *e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Space:
		break;
	default:
		break;
	}
}

void GLWidget3D::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

/**
* This function is called once before the first call to paintGL() or resizeGL().
*/
void GLWidget3D::initializeGL() {
	// init glew
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		std::cout << "Error: " << glewGetErrorString(err) << std::endl;
	}

	if (glewIsSupported("GL_VERSION_4_2"))
		printf("Ready for OpenGL 4.2\n");
	else {
		printf("OpenGL 4.2 not supported\n");
		exit(1);
	}
	const GLubyte* text = glGetString(GL_VERSION);
	printf("VERSION: %s\n", text);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_TEXTURE_2D);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	glTexGenf(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGenf(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glDisable(GL_TEXTURE_2D);

	glEnable(GL_TEXTURE_3D);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glDisable(GL_TEXTURE_3D);

	glEnable(GL_TEXTURE_2D_ARRAY);
	glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glDisable(GL_TEXTURE_2D_ARRAY);

	////////////////////////////////
	renderManager.init("", "", "", true, 8192);
	renderManager.resize(this->width(), this->height());

	glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex0"), 0);//tex0: 0
}

/**
* This function is called whenever the widget has been resized.
*/
void GLWidget3D::resizeGL(int width, int height) {
	height = height ? height : 1;
	glViewport(0, 0, width, height);
	camera.updatePMatrix(width, height);

	renderManager.resize(width, height);
}

/**
* This function is called whenever the widget needs to be painted.
*/
void GLWidget3D::paintEvent(QPaintEvent *event) {
	if (first_paint) {
		std::vector<Vertex> vertices;
		glutils::drawQuad(0.001, 0.001, glm::vec4(1, 1, 1, 1), glm::mat4(), vertices);
		renderManager.addObject("dummy", "", vertices, true);
		renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
		first_paint = false;
	}

	// draw by OpenGL
	makeCurrent();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	render();

	// unbind texture
	glActiveTexture(GL_TEXTURE0);

	// restore the settings for OpenGL
	glShadeModel(GL_FLAT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	QPainter painter(this);
	painter.setOpacity(1.0f);
	////
	painter.end();

	glEnable(GL_DEPTH_TEST);
}

/**
* This event handler is called when the mouse press events occur.
*/
void GLWidget3D::mousePressEvent(QMouseEvent *e) {
	// This is necessary to get key event occured even after the user selects a menu.
	setFocus();

	if (e->buttons() & Qt::LeftButton) {
		camera.mousePress(e->x(), e->y());
	}
	else if (e->buttons() & Qt::RightButton) {
	}
}

/**
* This event handler is called when the mouse move events occur.
*/

void GLWidget3D::mouseMoveEvent(QMouseEvent *e) {
	if (e->buttons() & Qt::LeftButton) {
		if (shiftPressed) {
			camera.move(e->x(), e->y());
		}
		else {
			camera.rotate(e->x(), e->y(), (ctrlPressed ? 0.1 : 1));
		}
	}

	update();
}

/**
* This event handler is called when the mouse release events occur.
*/
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e) {
}

void GLWidget3D::wheelEvent(QWheelEvent* e) {
	camera.zoom(e->delta() * 0.2);
	update();
}
