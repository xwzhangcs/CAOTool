#pragma once
#pragma warning(disable:4996)

#include <vector>
#include <map>
#include <tuple>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>

namespace util {

	// The following definitions are for triangulation only.
	struct FaceInfo {
		FaceInfo() {}
		int nesting_level;
		bool in_domain(){
			return nesting_level % 2 == 1;
		}
	};

	typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
	typedef CGAL::Triangulation_vertex_base_2<Kernel>                      Vb;
	typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, Kernel>    Fbb;
	typedef CGAL::Constrained_triangulation_face_base_2<Kernel, Fbb>        Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb>               TDS;
	typedef CGAL::Exact_predicates_tag                                Itag;
	typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag>  CDT;

	class PrimitiveShape {
	public:
		enum { TYPE_RECTANGLE, TYPE_TRIANGLE, TYPE_CURVE };

	public:
		cv::Mat_<float> mat;

	protected:
		PrimitiveShape() {}
		virtual ~PrimitiveShape() {}

	public:
		virtual boost::shared_ptr<PrimitiveShape> clone() const = 0;
		virtual std::vector<cv::Point2f> getActualPoints() const = 0;
		virtual int type() const = 0;
	};

	class PrimitiveRectangle : public PrimitiveShape {
	public:
		cv::Point2f min_pt;
		cv::Point2f max_pt;

	public:
		PrimitiveRectangle(const cv::Mat_<float>& mat, const cv::Point2f& min_pt, const cv::Point2f& max_pt);
		~PrimitiveRectangle() {}
		boost::shared_ptr<PrimitiveShape> clone() const;
		std::vector<cv::Point2f> getActualPoints() const;
		int type() const;
	};

	class PrimitiveTriangle : public PrimitiveShape {
	public:
		std::vector<cv::Point2f> points;

	public:
		PrimitiveTriangle(const cv::Mat_<float>& mat);
		PrimitiveTriangle(const cv::Mat_<float>& mat, const std::vector<cv::Point2f>& points);
		~PrimitiveTriangle() {}
		boost::shared_ptr<PrimitiveShape> clone() const;
		std::vector<cv::Point2f> getActualPoints() const;
		int type() const;
	};

	class PrimitiveCurve : public PrimitiveShape {
	public:
		float theta_start;
		float theta_end;
		cv::Point2f center;
		float radius;

	public:
		PrimitiveCurve(const cv::Mat_<float>& mat, float theta_start, float theta_end, const cv::Point2f &center, float radius);
		~PrimitiveCurve() {}
		boost::shared_ptr<PrimitiveShape> clone() const;
		std::vector<cv::Point2f> getActualPoints() const;
		int type() const;
	};

	class Ring {
	public:
		cv::Mat_<float> mat;
		std::vector<cv::Point2f> points;
		std::vector<int> pointsType;

	public:
		Ring();
		Ring(const Ring& ring);
		Ring(const std::vector<cv::Point2f>& points);

		Ring& operator=(const Ring& ring);
		Ring& operator=(const std::vector<cv::Point2f>& points);
		const cv::Point2f& front() const;
		cv::Point2f& front();
		const cv::Point2f& back() const;
		cv::Point2f& back();
		std::vector<cv::Point2f>::iterator begin();
		std::vector<cv::Point2f>::iterator end();
		size_t size() const;
		void clear();
		void resize(size_t s);
		const cv::Point2f& operator[](int index) const;
		cv::Point2f& operator[](int index);
		void push_back(const cv::Point2f& pt);
		void pop_back();
		void erase(std::vector<cv::Point2f>::iterator position);
		void translate(float x, float y);
		void transform(const cv::Mat_<float>& m);
		void clockwise();
		void counterClockwise();
		cv::Point2f getActualPoint(int index) const;
		Ring getActualPoints() const;
		bool isSimple() const;
	};

	class Polygon {
	public:
		cv::Mat_<float> mat;
		Ring contour;
		std::vector<Ring> holes;

		// For the right angle simplification, we currently store the decomposed primitive shapes in this data.
		// When generating a 3D geometry, or producing PLY file, this data should be used.
		std::vector<boost::shared_ptr<PrimitiveShape>> primitive_shapes;

	public:
		Polygon();

		Polygon clone() const;
		void translate(float x, float y);
		void transform(const cv::Mat_<float>& m);
		void clockwise();
		void counterClockwise();
	};

	bool isClockwise(const std::vector<cv::Point2f>& polygon);
	void clockwise(std::vector<cv::Point2f>& polygon);
	void counterClockwise(std::vector<cv::Point2f>& polygon);
	bool isSimple(const Polygon& polygon);
	bool isSimple(const Ring& points);
	bool isSimple(const std::vector<cv::Point>& points);
	void transform(std::vector<cv::Point2f>& polygon, const cv::Mat_<float>& m);
	std::vector<cv::Point> removeRedundantPoint(const std::vector<cv::Point>& polygon);
	Ring removeRedundantPoint(const Ring& polygon);
	//std::vector<cv::Point2f> removeRedundantPoint(const std::vector<cv::Point2f>& polygon);
	Ring resolveSelfIntersection(const Ring& ring);

	cv::Rect boundingBox(const std::vector<cv::Point>& polygon);
	cv::Rect boundingBox(const std::vector<cv::Point2f>& polygon);
	cv::Rect calculateOBB(const std::vector<cv::Point2f>& polygon, cv::Mat_<float>& trans_mat);
	bool withinPolygon(const cv::Point2f& pt, const Polygon& polygon);
	bool withinPolygon(const cv::Point2f& pt, const Ring& ring);
	bool withinPolygon(const Ring& inside_ring, const Ring& outside_ring);
	double calculateIOU(const cv::Mat_<uchar>& img, const cv::Mat_<uchar>& img2);
	double calculateIOU(const cv::Mat_<uchar>& img1, const cv::Mat_<uchar>& img2, const cv::Rect& rect);
	double calculateIOU(const Polygon& polygon1, const Polygon& polygon2);
	double calculateIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2);
	double calculateIOUbyCGAL(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2);
	double calculateIOUbyImage(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2, int image_size);
	double calculateIOU(const std::vector<Polygon>& polygons1, const std::vector<Polygon>& polygons2);
	double calculateArea(const Polygon& polygon);
	double calculateArea(const cv::Mat_<uchar>& img);
	double area(const CGAL::Polygon_with_holes_2<Kernel>& pwh);
	double area(const std::vector<cv::Point2f>& polygon);
	std::vector<Polygon> findContours(const cv::Mat_<uchar>& img, bool add_right_corner);
	std::vector<Polygon> findContours(const cv::Mat_<uchar>& image, int threshold, bool simplify, bool allow_diagonal, bool dilate);
	Ring addCornerToOpenCVContour(const std::vector<cv::Point>& polygon, const cv::Mat_<uchar>& img);
	void findContour(const cv::Mat_<uchar>& img, std::vector<cv::Point>& contour);
	void createImageFromContour(int width, int height, const std::vector<cv::Point>& contour, const cv::Point& offset, cv::Mat_<uchar>& result, bool erode = true);
	void createImageFromPolygon(int width, int height, const Polygon& polygon, const cv::Point& offset, cv::Mat_<uchar>& result);
	std::vector<util::Polygon> intersection(const util::Polygon& polygon1, const util::Polygon& polygon2);

	void approxPolyDP(const std::vector<cv::Point2f>& input_polygon, std::vector<cv::Point2f>& output_polygon, double epsilon, bool closed);

	void snapPolygon(const std::vector<util::Polygon>& ref_polygons, util::Polygon& polygon, float snapping_threshold);
	void snapPolygon(const std::vector<util::Polygon>& ref_polygons, std::vector<cv::Point2f>& polygon, float snapping_threshold);
	bool snapEdge(const cv::Point2f& p1, const cv::Point2f& p2, std::vector<cv::Point2f>& polygon, int i, int i2);
	void snapPolygon2(const std::vector<util::Polygon>& ref_polygons, util::Polygon& polygon, float snapping_threshold);
	void snapPolygon2(const std::vector<util::Polygon>& ref_polygons, std::vector<cv::Point2f>& polygon, float snapping_threshold);
	void snapPolygon3(const std::vector<util::Polygon>& ref_polygons, util::Polygon& polygon, float snapping_threshold);
	void snapPolygon3(const std::vector<util::Polygon>& ref_polygons, std::vector<cv::Point2f>& polygon, float snapping_threshold);
	bool snapEdge2(const cv::Point2f& p1, const cv::Point2f& p2, std::vector<cv::Point2f>& polygon, int i, int i2);
	float length(const cv::Point2f& pt);
	float length(const cv::Point3f& pt);
	float length(const cv::Point2f& p1, const cv::Point2f& p2);
	float dotProduct(const cv::Point2f& v1, const cv::Point2f& v2);
	float crossProduct(const cv::Point2f& v1, const cv::Point2f& v2);
	cv::Point3f crossProduct(const cv::Point3f& v1, const cv::Point3f& v2);
	float closestPoint(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segmentOnly, cv::Point2f& pt);
	float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, bool segmentOnly);
	float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d);
	bool segmentSegmentIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segmentOnly, cv::Point2f& intPoint);
	bool isTangent(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d);
	// tessellation
	std::vector<std::vector<cv::Point2f>> tessellate(const Ring& points);
	std::vector<std::vector<cv::Point2f>> tessellate(const Ring& points, const std::vector<Ring>& holes);
	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border);
	void mark_domains(CDT& cdt);

	bool lineLineIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segment_only, cv::Point2f& int_pt);

	float estimate(const std::vector<util::Polygon>& polygons);
	float estimate(const std::vector<cv::Point2f>& polygon);

	// score
	float calculateScoreRaOpt(const std::vector<cv::Point2f>& polygon, const std::vector<cv::Point2f>& init_polygon, int angle_threshold);
	float calculateScoreParallelOpt(const std::vector<cv::Point2f>& polygon, const std::vector<cv::Point2f>& init_polygon, int angle_threshold);
	float lineLineAngle(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d);
	float scoreFun(float angle, float threshold);
	// compute mirror point
	cv::Point2f mirrorPoint(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c);

	// score for point snap
	float calculateScorePointOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<cv::Point2f>& init_src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, const std::vector<std::vector<cv::Point2f>>& des_ini_layer_polygons, float dis_threshold);
	float calculateScoreSegOpt(const std::vector<cv::Point2f>& src_polygon, const std::vector<cv::Point2f>& init_src_polygon, const std::vector<std::vector<cv::Point2f>>& des_layer_polygons, const std::vector<std::vector<cv::Point2f>>& des_ini_layer_polygons, float dis_threshold, float angle_threshold);
}
