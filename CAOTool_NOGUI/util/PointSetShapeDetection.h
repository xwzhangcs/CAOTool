#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polygon_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "VerticalPlane.h"

namespace util {

	namespace pointcloud {

		namespace shape {

			// Type declarations
			typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
			typedef CGAL::Point_2<Kernel> Point2;
			typedef CGAL::Point_3<Kernel> Point3;
			typedef CGAL::Polygon_2<Kernel> Polygon2;
			typedef CGAL::Polygon_with_holes_2<Kernel> PolygonWithHoles2;

			typedef Kernel::FT                                           FT;
			typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
			typedef std::vector<Point_with_normal>                       Pwn_vector;
			typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
			typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

			// In Shape_detection_traits the basic types, i.e., Point and Vector types
			// as well as iterator type and property maps, are defined.
			//typedef CGAL::Shape_detection_3::Shape_detection_traits<Kernel, Pwn_vector, Point_map, Normal_map> Traits;
			typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map> Traits;
			typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
			typedef CGAL::Shape_detection_3::Cylinder<Traits>			 Cylinder;
			typedef VerticalPlane<Traits>								 VPlane;

			typedef CGAL::Point_set_3<Kernel::Point_3> Point_set;

			bool detectCurve(std::vector<std::pair<cv::Point3f, cv::Point3f>>& point_cloud, double probability, int min_points, double epsilon, double cluster_epsilon, double normal_threshold, double min_radius, double max_radius);

		}
	}

}