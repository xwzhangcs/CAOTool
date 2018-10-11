#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <fstream> 
#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "GLWidget3D.h"

class MainWindow : public QMainWindow {
	Q_OBJECT

private:
	Ui::MainWindowClass ui;
	GLWidget3D* glWidget;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
	// Helper functions
	int indexOfNumberLetter(std::string& str, int offset);
	int lastIndexOfNumberLetter(std::string& str);
	std::vector<std::string> split(const std::string &s, char delim);
	std::vector<std::string> loadOBJ(std::string filename, std::string save_path);
	std::vector<int> FindMatachingLayers(std::vector<std::string> jido_layer_images, std::vector<std::string> cgv_layer_images);

public slots:
	void onOpen();
	void onSaveOBJ();
	void onLoadOBJ();
	void onLoadOBJ_new();
	void onSaveTopFaces();
	void onSavePLY();
	void onSaveImage();
	void onInputVoxel();
	void onSimplifyByAll();
	void onSimplifyByDP();
	void onSimplifyByRightAngle();
	void onSimplifyByCurve();
	void onSimplifyByCurveRightAngle();
	void onSimplifyByEfficientRANSAC();
	void onOffsetScale();
	void onColoringModeChanged();
	void onRenderingModeChanged();
	void onFindMatachingLayers();
	void onCompareModels();
};

#endif // MAINWINDOW_H
