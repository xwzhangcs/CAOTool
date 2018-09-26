#include "MainWindow.h"
#include <QFileDialog>
#include <QDateTime>
#include <QMessageBox>
#include "AllOptionDialog.h"
#include "DPOptionDialog.h"
#include "RightAngleOptionDialog.h"
#include "CurveOptionDialog.h"
#include "CurveRightAngleOptionDialog.h"
#include "EfficientRANSACOptionDialog.h"
#include "OffsetScaleDialog.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	// group for simplification modes
	QActionGroup* groupSimplify = new QActionGroup(this);
	groupSimplify->addAction(ui.actionInputVoxel);
	groupSimplify->addAction(ui.actionSimplifyByAll);
	groupSimplify->addAction(ui.actionSimplifyByDP);
	groupSimplify->addAction(ui.actionSimplifyByRightAngle);
	groupSimplify->addAction(ui.actionSimplifyByCurve);
	groupSimplify->addAction(ui.actionSimplifyByCurveRightAngle);

	// group for rendering modes
	QActionGroup* groupColoring = new QActionGroup(this);
	groupColoring->addAction(ui.actionColor);
	groupColoring->addAction(ui.actionTexture);

	// group for rendering modes
	QActionGroup* groupRendering = new QActionGroup(this);
	groupRendering->addAction(ui.actionRenderingBasic);
	groupRendering->addAction(ui.actionRenderingSSAO);
	groupRendering->addAction(ui.actionRenderingHatching);

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionSaveOBJ, SIGNAL(triggered()), this, SLOT(onSaveOBJ()));
	connect(ui.actionLoadOBJ, SIGNAL(triggered()), this, SLOT(onLoadOBJ()));
	connect(ui.actionSaveTopFaces, SIGNAL(triggered()), this, SLOT(onSaveTopFaces()));
	connect(ui.actionSavePLY, SIGNAL(triggered()), this, SLOT(onSavePLY()));
	connect(ui.actionSaveImage, SIGNAL(triggered()), this, SLOT(onSaveImage()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionInputVoxel, SIGNAL(triggered()), this, SLOT(onInputVoxel()));
	connect(ui.actionSimplifyByAll, SIGNAL(triggered()), this, SLOT(onSimplifyByAll()));
	connect(ui.actionSimplifyByDP, SIGNAL(triggered()), this, SLOT(onSimplifyByDP()));
	connect(ui.actionSimplifyByRightAngle, SIGNAL(triggered()), this, SLOT(onSimplifyByRightAngle()));
	connect(ui.actionSimplifyByCurve, SIGNAL(triggered()), this, SLOT(onSimplifyByCurve()));
	connect(ui.actionSimplifyByCurveRightAngle, SIGNAL(triggered()), this, SLOT(onSimplifyByCurveRightAngle()));
	connect(ui.actionSimplifyByEfficientRANSAC, SIGNAL(triggered()), this, SLOT(onSimplifyByEfficientRANSAC()));
	connect(ui.actionOffsetScale, SIGNAL(triggered()), this, SLOT(onOffsetScale()));
	connect(ui.actionColor, SIGNAL(triggered()), this, SLOT(onColoringModeChanged()));
	connect(ui.actionTexture, SIGNAL(triggered()), this, SLOT(onColoringModeChanged()));
	connect(ui.actionRenderingBasic, SIGNAL(triggered()), this, SLOT(onRenderingModeChanged()));
	connect(ui.actionRenderingSSAO, SIGNAL(triggered()), this, SLOT(onRenderingModeChanged())); 
	connect(ui.actionRenderingHatching, SIGNAL(triggered()), this, SLOT(onRenderingModeChanged()));
	connect(ui.actionFindMatachingLayers, SIGNAL(triggered()), this, SLOT(onFindMatachingLayers()));

	// create tool bar for file menu
	ui.mainToolBar->addAction(ui.actionOpen);
	ui.mainToolBar->addAction(ui.actionSaveOBJ);

	// setup the GL widget
	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

MainWindow::~MainWindow() {
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Load voxel data..."), "", tr("Image files (*.png *.jpg *.bmp)"));
	if (filename.isEmpty()) return;

	setWindowTitle("LEGO - " + filename);
	glWidget->loadVoxelData(filename);
	glWidget->update();
}

void MainWindow::onFindMatachingLayers(){
	// get layer images for jido model
	QString filename_jido = QFileDialog::getOpenFileName(this, tr("Load voxel data..."), "", tr("Image files (*.png *.jpg *.bmp)"));
	if (filename_jido.isEmpty()) return;
	// get directory
	QDir dir = QFileInfo(filename_jido).absoluteDir();
	// scan all the files in the directory to get a voxel data
	QStringList files_jido = dir.entryList(QDir::NoDotAndDotDot | QDir::Files, QDir::DirsFirst);
	std::vector<cv::Mat_<uchar>> layer_imgs_jido(files_jido.size());
	for (int i = 0; i < files_jido.size(); i++) {
		std::cout << (dir.absolutePath() + "/" + files_jido[i]).toUtf8().constData() << std::endl;
		layer_imgs_jido[i] = cv::imread((dir.absolutePath() + "/" + files_jido[i]).toUtf8().constData(), cv::IMREAD_GRAYSCALE);
	}
	// get layer images for cgv model
	QString filename_cgv = QFileDialog::getOpenFileName(this, tr("Load voxel data..."), "", tr("Image files (*.png *.jpg *.bmp)"));
	if (filename_cgv.isEmpty()) return;
	// get directory
	dir = QFileInfo(filename_cgv).absoluteDir();
	// scan all the files in the directory to get a voxel data
	QStringList files_cgv = dir.entryList(QDir::NoDotAndDotDot | QDir::Files, QDir::DirsFirst);
	std::vector<cv::Mat_<uchar>> layer_imgs_cgv(files_cgv.size());
	for (int i = 0; i < files_cgv.size(); i++) {
		std::cout << (dir.absolutePath() + "/" + files_cgv[i]).toUtf8().constData() << std::endl;
		layer_imgs_cgv[i] = cv::imread((dir.absolutePath() + "/" + files_cgv[i]).toUtf8().constData(), cv::IMREAD_GRAYSCALE);
	}
	
	// get new image size 
	int rows_jido = layer_imgs_jido[0].rows;
	int cols_jido = layer_imgs_jido[0].cols;
	std::cout << "layer_imgs_cgv.size() is " << layer_imgs_cgv.size() << std::endl;
	// resize cgv images
	for (int i = 0; i < layer_imgs_cgv.size(); i++){
		cv::Mat img_tmp;
		cv::resize(layer_imgs_cgv[i], img_tmp, cv::Size(cols_jido, rows_jido), 0, 0, cv::INTER_NEAREST);
		layer_imgs_cgv[i] = img_tmp;
		//QString img_filename = "../data/cgv_" + QString::number(i) + ".png";
		//cv::imwrite(img_filename.toUtf8().constData(), layer_imgs_cgv[i]);
	}

	// find intersection between jido and cgv models
	std::vector<int> match_layer_indices;
	for (int i = 0; i < layer_imgs_cgv.size(); i++){
		cv::Mat_<uchar> img2 = layer_imgs_cgv[i];
		float best_score = -std::numeric_limits<float>::max();
		int best_index = -1;
		for (int j = 0; j < layer_imgs_jido.size(); j++){
			// computer IOU
			float score = 0.0f;
			int inter_cnt = 0;
			int union_cnt = 0;
			cv::Mat_<uchar> img1 = layer_imgs_jido[j];
			for (int r = 0; r < img1.rows; r++) {
				for (int c = 0; c < img1.cols; c++) {
					if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
					if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
				}
			}
			if (union_cnt == 0){
				score = 0.0f;
			}
			else
				score = (double)inter_cnt / union_cnt;
			if (score > best_score){
				best_score = score;
				best_index = j;
			}

		}
		std::cout << "best_score is " << best_score << std::endl;
		std::cout << "best_index is " << best_index << std::endl;
		match_layer_indices.push_back(best_index);
		// find the maximum
		
	}
}

void MainWindow::onSaveOBJ() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save OBJ file..."), "", tr("OBJ files (*.obj)"));
	if (!filename.isEmpty()) {
		glWidget->saveOBJ(filename);
	}
}

int MainWindow::indexOfNumberLetter(std::string& str, int offset) {
	for (int i = offset; i < str.length(); ++i) {
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '-' || str[i] == '.') return i;
	}
	return str.length();
}

int MainWindow::lastIndexOfNumberLetter(std::string& str) {
	for (int i = str.length() - 1; i >= 0; --i) {
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '-' || str[i] == '.') return i;
	}
	return 0;
}

std::vector<std::string> MainWindow::split(const std::string &s, char delim) {
	std::vector<std::string> elems;

	std::stringstream ss(s);
	std::string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}

	return elems;
}

void MainWindow::onLoadOBJ() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Load OBJ file..."), "", tr("OBJ files (*.obj)"));
	if (!filename.isEmpty()) {
		std::ifstream file(filename.toUtf8().constData());
		if (!file.is_open()) {
			return;
		}

		// Store vertex
		std::vector<glm::vec3> raw_vertices;
		std::vector<std::pair<float, glm::vec2>> layer_vertices;
		std::vector<glm::vec3> face_vertex_indices;
		std::vector<float> layer_heights;

		std::string line;
		while (getline(file, line)) {
			if (line.substr(0, 2) == "v ") {
				// Read position data
				int index1 = indexOfNumberLetter(line, 2);
				int index2 = lastIndexOfNumberLetter(line);
				std::vector<std::string> values = split(line.substr(index1, index2 - index1 + 1), ' ');
				glm::vec3 vert(stof(values[0]), stof(values[1]), stof(values[2]));
				glm::vec2 vert2d(stof(values[0]), stof(values[1]));
				float z_value = stof(values[2]);
				//std::cout << "x is " << vert.x << ", y is " << vert.y << ", z is " << vert.z << std::endl;
				raw_vertices.push_back(vert);
				layer_vertices.push_back(std::make_pair(z_value, vert2d ));
			}
			else if (line.substr(0, 3) == "vn ") {
				// Read normal data

			}
			else if (line.substr(0, 2) == "f ") {
				// Read face data
				// Read face data
				int index1 = indexOfNumberLetter(line, 2);
				int index2 = lastIndexOfNumberLetter(line);
				std::vector<std::string> values = split(line.substr(index1, index2 - index1 + 1), ' ');
				for (int i = 0; i < values.size() - 2; i++) {
					// Split up vertex indices
					std::vector<std::string> v1 = split(values[0], '/');		// Triangle fan for ngons
					std::vector<std::string> v2 = split(values[i + 1], '/');
					std::vector<std::string> v3 = split(values[i + 2], '/');

					// Store position indices
					glm::vec3 vert(stoi(v1[0]) - 1, stoi(v2[0]) - 1, stoi(v3[0]) - 1);
					face_vertex_indices.push_back(vert);
					//std::cout << "1st index is " << vert.x << ", 2nd is " << vert.y << ", 3rd is " << vert.z << std::endl;
				}
			}
		}
		file.close();
		
		//std::cout << "layer_vertices size is " << layer_vertices.size() << std::endl;
		sort(layer_vertices.begin(), layer_vertices.end(), [](const std::pair<float, glm::vec2>& lhs, std::pair<float, glm::vec2>& rhs){return lhs.first < rhs.first; });
		for (int i = 0; i < layer_vertices.size(); i++)
		{
			if (layer_heights.size() == 0)
				layer_heights.push_back(layer_vertices[i].first);
			else{
				if (layer_vertices[i].first - layer_heights[layer_heights.size() - 1] >= 0.6)
					layer_heights.push_back(layer_vertices[i].first);
			}
		}
		// output layer height
		for (int i = 0; i < layer_heights.size(); i++){
			std::cout << "layer " << i <<" height is " << layer_heights[i] << std::endl;
		}
		// earse the ground layer
		layer_heights.erase(layer_heights.begin());
		//
		std::vector<std::vector<glm::vec3>> layer_faces;
		layer_faces.resize(layer_heights.size());
		for (int i = 0; i < face_vertex_indices.size(); i++){
			int first = face_vertex_indices[i].x;
			int second = face_vertex_indices[i].y;
			int third = face_vertex_indices[i].z;
			//if (raw_vertices[first].z == raw_vertices[second].z && raw_vertices[first].z == raw_vertices[third].z){
			if (abs(raw_vertices[first].z - raw_vertices[second].z) < 0.6 && abs(raw_vertices[first].z - raw_vertices[third].z) < 0.6){
				for (int j = 0; j < layer_heights.size(); j++){
					//if (raw_vertices[first].z - layer_heights[j] < 0.6)
					if (raw_vertices[first].z >= layer_heights[j])
						layer_faces[j].push_back(face_vertex_indices[i]);
				}
			}
		}
		// get min_x min_y max_x max_y
		float min_x = std::numeric_limits<float>::max();
		float min_y = std::numeric_limits<float>::max();
		float max_x = -std::numeric_limits<float>::max();
		float max_y = -std::numeric_limits<float>::max();

		for (int i = 0; i < raw_vertices.size(); i++) {
			min_x = std::min(min_x, raw_vertices[i].x);
			min_y = std::min(min_y, raw_vertices[i].y);
			max_x = std::max(max_x, raw_vertices[i].x);
			max_y = std::max(max_y, raw_vertices[i].y);
		}
		min_x = min_x - 5;
		min_y = min_y - 5;
		max_x = max_x + 5;
		max_y = max_y + 5;
		int rows = max_y - min_y;
		int cols = max_x - min_x;
		int lineType = 8;
		for (int i = 0; i < layer_faces.size(); i++){
			cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC3);
			std::cout << "layer faces " << i << ", size is " << layer_faces[i].size() << std::endl;
			for (int j = 0; j < layer_faces[i].size(); j++){
				cv::Point rook_points[1][3] = { 0 };
				const cv::Point* ppt[1];
				rook_points[0][0] = cv::Point(raw_vertices[layer_faces[i][j].x].x - min_x, raw_vertices[layer_faces[i][j].x].y - min_y);
				rook_points[0][1] = cv::Point(raw_vertices[layer_faces[i][j].y].x - min_x, raw_vertices[layer_faces[i][j].y].y - min_y);
				rook_points[0][2] = cv::Point(raw_vertices[layer_faces[i][j].z].x - min_x, raw_vertices[layer_faces[i][j].z].y - min_y);
				int npt[1] = {3};
				ppt[0] = rook_points[0];
				/*std::cout << "rook_points[0][0] " << rook_points[0][0] << std::endl;
				std::cout << "rook_points[0][1] " << rook_points[0][1] << std::endl;
				std::cout << "rook_points[0][2] " << rook_points[0][2] << std::endl;*/
				//std::cout << "----" << std::endl;
				cv::fillPoly(img,
					ppt,
					npt,
					1,
					cv::Scalar(255, 255, 255),
					lineType);
			}

			QString img_filename = "../data/" + QString::number(i) + ".png";
			cv::imwrite(img_filename.toUtf8().constData(), img);
		}

		/*
		// extract contour
		for (int i = 0; i < layer_faces.size(); i++){
			QString img_filename = "../data/" + QString::number(i) + ".png";
			cv::Mat src = cv::imread(img_filename.toUtf8().constData(), 0);
			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			/// Find contours
			findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
			cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC3);
			std::cout << "contours size is " << contours.size() << std::endl;
			for (int j = 0; j < contours.size(); j++){
				drawContours(img, contours, j, cv::Scalar(255, 255, 255), 1, 8, hierarchy, 0, cv::Point());
			}
			std::cout << "Before contour size is " << contours[0].size() << std::endl;
			std::vector<cv::Point2f> contour;
			float epsilon = 1.0f;
			cv::approxPolyDP(contours[0], contour, epsilon, true);
			std::cout << "Before contour size is " << contour.size() << std::endl;
			img_filename = "../data/contour_" + QString::number(i) + ".png";
			cv::imwrite(img_filename.toUtf8().constData(), img);
		}*/

	}
}

void MainWindow::onLoadOBJ_new() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Load OBJ file..."), "", tr("OBJ files (*.obj)"));
	if (!filename.isEmpty()) {
		std::ifstream file(filename.toUtf8().constData());
		if (!file.is_open()) {
			return;
		}

		// Store vertex
		std::vector<glm::vec3> raw_vertices;
		std::vector<std::pair<float, glm::vec2>> layer_vertices;
		std::vector<glm::vec3> face_vertex_indices;
		std::vector<float> layer_heights;

		std::string line;
		while (getline(file, line)) {
			if (line.substr(0, 2) == "v ") {
				// Read position data
				int index1 = indexOfNumberLetter(line, 2);
				int index2 = lastIndexOfNumberLetter(line);
				std::vector<std::string> values = split(line.substr(index1, index2 - index1 + 1), ' ');
				glm::vec3 vert(stof(values[0]), stof(values[1]), stof(values[2]));
				glm::vec2 vert2d(stof(values[0]), stof(values[1]));
				float z_value = stof(values[2]);
				//std::cout << "x is " << vert.x << ", y is " << vert.y << ", z is " << vert.z << std::endl;
				raw_vertices.push_back(vert);
				layer_vertices.push_back(std::make_pair(z_value, vert2d));
			}
			else if (line.substr(0, 3) == "vn ") {
				// Read normal data

			}
			else if (line.substr(0, 2) == "f ") {
				// Read face data
				// Read face data
				int index1 = indexOfNumberLetter(line, 2);
				int index2 = lastIndexOfNumberLetter(line);
				std::vector<std::string> values = split(line.substr(index1, index2 - index1 + 1), ' ');
				for (int i = 0; i < values.size() - 2; i++) {
					// Split up vertex indices
					std::vector<std::string> v1 = split(values[0], '/');		// Triangle fan for ngons
					std::vector<std::string> v2 = split(values[i + 1], '/');
					std::vector<std::string> v3 = split(values[i + 2], '/');

					// Store position indices
					glm::vec3 vert(stoi(v1[0]) - 1, stoi(v2[0]) - 1, stoi(v3[0]) - 1);
					face_vertex_indices.push_back(vert);
					//std::cout << "1st index is " << vert.x << ", 2nd is " << vert.y << ", 3rd is " << vert.z << std::endl;
				}
			}
		}
		file.close();

		//std::cout << "layer_vertices size is " << layer_vertices.size() << std::endl;
		sort(layer_vertices.begin(), layer_vertices.end(), [](const std::pair<float, glm::vec2>& lhs, std::pair<float, glm::vec2>& rhs){return lhs.first < rhs.first; });
		for (int i = 0; i < layer_vertices.size(); i++)
		{
			if (layer_heights.size() == 0)
				layer_heights.push_back(layer_vertices[i].first);
			else{
				if (layer_vertices[i].first - layer_heights[layer_heights.size() - 1] >= 0.6)
					layer_heights.push_back(layer_vertices[i].first);
			}
		}
		// output layer height
		for (int i = 0; i < layer_heights.size(); i++){
			std::cout << "layer " << i << " height is " << layer_heights[i] << std::endl;
		}
		//
		std::vector<std::vector<glm::vec3>> layer_faces;
		layer_faces.resize(layer_heights.size());
		for (int i = 0; i < face_vertex_indices.size(); i++){
			int first = face_vertex_indices[i].x;
			int second = face_vertex_indices[i].y;
			int third = face_vertex_indices[i].z;
			//if (raw_vertices[first].z == raw_vertices[second].z && raw_vertices[first].z == raw_vertices[third].z){
			if (abs(raw_vertices[first].z - raw_vertices[second].z) < 0.6 && abs(raw_vertices[first].z - raw_vertices[third].z) < 0.6){
				for (int j = 0; j < layer_heights.size(); j++){
					if (raw_vertices[first].z - layer_heights[j] < 0.6)
					//if (raw_vertices[first].z >= layer_heights[j])
						layer_faces[j].push_back(face_vertex_indices[i]);
				}
			}
		}
		// get min_x min_y max_x max_y
		float min_x = std::numeric_limits<float>::max();
		float min_y = std::numeric_limits<float>::max();
		float max_x = -std::numeric_limits<float>::max();
		float max_y = -std::numeric_limits<float>::max();

		for (int i = 0; i < raw_vertices.size(); i++) {
			min_x = std::min(min_x, raw_vertices[i].x);
			min_y = std::min(min_y, raw_vertices[i].y);
			max_x = std::max(max_x, raw_vertices[i].x);
			max_y = std::max(max_y, raw_vertices[i].y);
		}
		min_x = min_x - 5;
		min_y = min_y - 5;
		max_x = max_x + 5;
		max_y = max_y + 5;
		int rows = max_y - min_y;
		int cols = max_x - min_x;
		int lineType = 8;
		for (int i = 1; i < 2/*layer_faces.size()*/; i++){
			cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC3);
			std::cout << "layer faces " << i << ", size is " << layer_faces[i].size() << std::endl;
			for (int j = 0; j < 2; j++){
				cv::Point rook_points[1][3] = { 0 };
				const cv::Point* ppt[1];
				rook_points[0][0] = cv::Point(raw_vertices[layer_faces[i][j].x].x - min_x, raw_vertices[layer_faces[i][j].x].y - min_y);
				rook_points[0][1] = cv::Point(raw_vertices[layer_faces[i][j].y].x - min_x, raw_vertices[layer_faces[i][j].y].y - min_y);
				rook_points[0][2] = cv::Point(raw_vertices[layer_faces[i][j].z].x - min_x, raw_vertices[layer_faces[i][j].z].y - min_y);
				int npt[1] = { 3 };
				ppt[0] = rook_points[0];
				printf("%f\n", raw_vertices[layer_faces[i][j].x].z);
				std::cout << "rook_points[0][0] (" << rook_points[0][0] << std::endl;
				printf("%f\n", raw_vertices[layer_faces[i][j].y].z);
				std::cout << "rook_points[0][1] (" << rook_points[0][1] << std::endl;
				printf("%f\n", raw_vertices[layer_faces[i][j].z].z);
				std::cout << "rook_points[0][2] (" << rook_points[0][2] << std::endl;
				std::cout << "----" << std::endl;
				cv::fillPoly(img,
					ppt,
					npt,
					1,
					cv::Scalar(255, 255, 255),
					lineType);
			}

			/*{
			std::cout << "layer faces " << i << ", size is " << layer_faces[i].size() << std::endl;
			cv::Point rook_points[2][3];
			rook_points[0][0] = cv::Point(raw_vertices[layer_faces[i][2].x].x - min_x, raw_vertices[layer_faces[i][2].x].y - min_y);
			rook_points[0][1] = cv::Point(raw_vertices[layer_faces[i][2].y].x - min_x, raw_vertices[layer_faces[i][2].y].y - min_y);
			rook_points[0][2] = cv::Point(raw_vertices[layer_faces[i][2].z].x - min_x, raw_vertices[layer_faces[i][2].z].y - min_y);
			rook_points[1][0] = cv::Point(raw_vertices[layer_faces[i][3].x].x - min_x, raw_vertices[layer_faces[i][3].x].y - min_y);
			rook_points[1][1] = cv::Point(raw_vertices[layer_faces[i][3].y].x - min_x, raw_vertices[layer_faces[i][3].y].y - min_y);
			rook_points[1][2] = cv::Point(raw_vertices[layer_faces[i][3].z].x - min_x, raw_vertices[layer_faces[i][3].z].y - min_y);
			std::cout << "rook_points[0][0] " << rook_points[0][0] << std::endl;
			std::cout << "rook_points[0][1] " << rook_points[0][1] << std::endl;
			std::cout << "rook_points[0][2] " << rook_points[0][2] << std::endl;
			std::cout << "rook_points[1][0] " << rook_points[1][0] << std::endl;
			std::cout << "rook_points[1][1] " << rook_points[1][1] << std::endl;
			std::cout << "rook_points[1][2] " << rook_points[1][2] << std::endl;
			const cv::Point* ppt[2] = { rook_points[0], rook_points[1] };
			int npt[] = { 3, 3 };
			cv::fillPoly(img,
			ppt,
			npt,
			2,
			cv::Scalar(255, 255, 255),
			lineType);
			}*/
			QString img_filename = "../data/" + QString::number(i) + ".png";
			cv::imwrite(img_filename.toUtf8().constData(), img);
		}

		
		// extract contour
		//for (int i = 1; i < 2/*layer_faces.size()*/; i++){
		//QString img_filename = "../data/" + QString::number(i) + ".png";
		//cv::Mat src = cv::imread(img_filename.toUtf8().constData(), 0);
		//std::vector<std::vector<cv::Point> > contours;
		//std::vector<cv::Vec4i> hierarchy;
		///// Find contours
		//findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		//cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC3);
		//std::cout << "contours size is " << contours.size() << std::endl;
		//for (int j = 0; j < contours.size(); j++){
		//drawContours(img, contours, j, cv::Scalar(255, 255, 255), 1, 8, hierarchy, 0, cv::Point());
		//}
		//std::cout << "Before contour size is " << contours[0].size() << std::endl;
		//std::vector<cv::Point2f> contour;
		//float epsilon = 1.0f;
		//cv::approxPolyDP(contours[0], contour, epsilon, true);
		//std::cout << "Before contour size is " << contour.size() << std::endl;
		//for (int j = 0; j < contour.size(); j++)
		//	std::cout << contour[j] << std::endl;
		//img_filename = "../data/contour_" + QString::number(i) + ".png";
		//cv::imwrite(img_filename.toUtf8().constData(), img);
		//}

	}
}


void MainWindow::onSaveTopFaces() {
	if (glWidget->show_mode == GLWidget3D::SHOW_INPUT) {
		QMessageBox msg;
		msg.setText("Simplify the buildings.");
		msg.exec();
		return;
	}

	QString filename = QFileDialog::getSaveFileName(this, tr("Save text file..."), "", tr("text files (*.txt)"));
	if (!filename.isEmpty()) {
		glWidget->saveTopFace(filename);
	}
}

void MainWindow::onSavePLY() {
	if (glWidget->show_mode == GLWidget3D::SHOW_INPUT) {
		QMessageBox msg;
		msg.setText("Simplify the buildings.");
		msg.exec();
		return;
	}

	QString filename = QFileDialog::getSaveFileName(this, tr("Save PLY file..."), "", tr("PLY files (*.ply)"));
	if (!filename.isEmpty()) {
		glWidget->savePLY(filename);
	}
}

void MainWindow::onSaveImage() {
	if (!QDir("screenshot").exists()) {
		QDir().mkdir("screenshot");
	}
	QDateTime dateTime = QDateTime().currentDateTime();
	QString str = QString("screenshot/") + dateTime.toString("yyyyMMddhhmmss") + QString(".png");

	glWidget->saveImage(str);
}

void MainWindow::onInputVoxel() {
	glWidget->showInputVoxel();
	glWidget->update();
}

void MainWindow::onSimplifyByAll() {
	AllOptionDialog dlg;
	if (dlg.exec()) {
		glWidget->simplifyByAll(dlg.getAlpha());
		glWidget->update();
	}
}

void MainWindow::onSimplifyByDP() {
	DPOptionDialog dlg;
	if (dlg.exec()) {
		glWidget->simplifyByDP(dlg.getEpsilon(), dlg.getLayeringThreshold(), dlg.getSnappingThreshold(), dlg.getOrientation() / 180.0 * CV_PI, dlg.getMinContourArea(), dlg.getMaxOBBRatio(), dlg.isAllowTriangleContour(), dlg.isAllowOverhang());
		glWidget->update();
	}
}

void MainWindow::onSimplifyByRightAngle() {
	RightAngleOptionDialog dlg;
	if (dlg.exec()) {
		glWidget->simplifyByRightAngle(dlg.getResolution(), dlg.getOptimization(), dlg.getLayeringThreshold(), dlg.getSnappingThreshold(), dlg.getOrientation() / 180.0 * CV_PI, dlg.getMinContourArea(), dlg.getMaxOBBRatio(), dlg.isAllowTriangleContour(), dlg.isAllowOverhang());
		glWidget->update();
	}
}

void MainWindow::onSimplifyByCurve() {
	CurveOptionDialog dlg;
	if (dlg.exec()) {
		glWidget->simplifyByCurve(dlg.getEpsilon(), dlg.getCurveThreshold(), dlg.getLayeringThreshold(), dlg.getSnappingThreshold(), dlg.getOrientation() / 180.0 * CV_PI, dlg.getMinContourArea(), dlg.getMaxOBBRatio(), dlg.isAllowTriangleContour(), dlg.isAllowOverhang());
		glWidget->update();
	}
}

void MainWindow::onSimplifyByCurveRightAngle() {
	CurveRightAngleOptionDialog dlg;
	if (dlg.exec()) {
		glWidget->simplifyByCurveRightAngle(dlg.getEpsilon(), dlg.getCurveThreshold(), dlg.getAngleThreshold() / 180.0 * CV_PI, dlg.getLayeringThreshold(), dlg.getSnappingThreshold(), dlg.getOrientation() / 180.0 * CV_PI, dlg.getMinContourArea(), dlg.getMaxOBBRatio(), dlg.isAllowTriangleContour(), dlg.isAllowOverhang());
		glWidget->update();
	}
}

void MainWindow::onSimplifyByEfficientRANSAC() {
	EfficientRANSACOptionDialog dlg;
	if (dlg.exec()) {
		glWidget->simplifyByEfficientRansac(dlg.getCurveNumIterations(), dlg.getCurveMinPoints(), dlg.getCurveMaxErrorRatioToRadius(), dlg.getCurveClusterEpsilon(), dlg.getCurveMinAngle() / 180.0 * CV_PI, dlg.getCurveMinRadius(), dlg.getCurveMaxRadius(), dlg.getLineNumIterations(), dlg.getLineMinPoints() * 0.01, dlg.getLineMaxError(), dlg.getLineClusterEpsilon() * 0.01, dlg.getLineMinLength() * 0.01, dlg.getLineAngleThreshold() / 180.0 * CV_PI, dlg.getContourMaxError(), dlg.getContourAngleThreshold() / 180.0 * CV_PI, dlg.getLayeringThreshold(), dlg.getSnappingThreshold(), dlg.getOrientation() / 180.0 * CV_PI, dlg.getMinContourArea(), dlg.getMaxOBBRatio(), dlg.isAllowTriangleContour(), dlg.isAllowOverhang(), dlg.getConfigFile());
	}
}

void MainWindow::onOffsetScale() {
	OffsetScaleDialog dlg;
	dlg.setOffset(glWidget->offset.x, glWidget->offset.y, glWidget->offset.z);
	dlg.setScale(glWidget->scale);
	if (dlg.exec()) {
		glWidget->offset = glm::dvec3(dlg.getOffsetX(), dlg.getOffsetY(), dlg.getOffsetZ());
		glWidget->scale = dlg.getScale();
		glWidget->update3DGeometry();
		glWidget->update();
	}
}

void MainWindow::onColoringModeChanged() {
	if (ui.actionColor->isChecked()) {
		glWidget->color_mode = GLWidget3D::COLOR;
	}
	else if (ui.actionTexture->isChecked()) {
		glWidget->color_mode = GLWidget3D::TEXTURE;
	}
	glWidget->update3DGeometry();
	glWidget->update();
}

void MainWindow::onRenderingModeChanged() {
	if (ui.actionRenderingBasic->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_BASIC;
	}
	else if (ui.actionRenderingSSAO->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_SSAO;
	}
	else if (ui.actionRenderingHatching->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_HATCHING;
	}
	glWidget->update();
}