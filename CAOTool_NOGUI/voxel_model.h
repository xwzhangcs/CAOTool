#pragma once

#include <iostream>
#include <QDir>
#include <QString>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace voxel_model {

	class Point3d {
	public:
		double x;
		double y;
		double z;

	public:
		Point3d() {}
		Point3d(double x, double y, double z) : x(x), y(y), z(z) {}
	};

	class Triangle {
	public:
		int a, b, c;

	public:
		Triangle(int a, int b, int c) :a(a), b(b), c(c) {}
	};

	void put_voxel(int x, int y, int z, std::vector<Point3d> &vertices, std::vector<Triangle> &faces);
	int voxel_model(std::vector<cv::Mat_<uchar>> &voxel_data, std::string output_ply_filename, double offset_x, double offset_y, double offset_z, double scale);

}
