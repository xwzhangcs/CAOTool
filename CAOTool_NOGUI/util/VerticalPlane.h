#pragma once

#include <CGAL/Shape_detection_3.h>
#include <CGAL/number_utils.h>

namespace util {

	namespace pointcloud {

		namespace shape {

			template <class Traits>
			class VerticalPlane : public CGAL::Shape_detection_3::Shape_base < Traits > {
			public:
				typedef typename Traits::Point_map Point_map;
				typedef typename Traits::Normal_map Normal_map;
				typedef typename Traits::FT FT; ///< number type.
				typedef typename Traits::Point_3 Point_3; ///< point type.
				typedef typename Traits::Point_2 Point_2; ///< point 2D type.
				typedef typename Traits::Vector_3 Vector_3;
				typedef typename Traits::Plane_3 Plane_3;///< plane type for conversion operator.

			public:
				VerticalPlane() : CGAL::Shape_detection_3::Shape_base<Traits>() {}

				operator Plane_3() const {
					return Plane_3(this->get_x(m_normal), this->get_y(m_normal), this->get_z(m_normal), m_d);
				}

				Vector_3 plane_normal() const {
					return m_normal;
				}

				FT d() const {
					return m_d;
				}

				//  Computes squared Euclidean distance from query point to the shape.
				FT squared_distance(const Point_3 &p) const {
					FT d = this->scalar_pdct(this->constr_vec(p, m_point_on_primitive), m_normal);
					return d * d;
				}

				/*!
				Computes the orthogonal projection of a query point on the shape.
				*/
				Point_3 projection(const Point_3& p) const {
					return to_3d(to_2d(p));
				}

				Point_2 to_2d(const Point_3& p) const {
					Vector_3 v(m_point_on_primitive, p);
					return Point_2(v * m_base1, v * m_base2);
				}

				Point_3 to_3d(const Point_2& p) const {
					return m_point_on_primitive + p.x() * m_base1 + p.y() * m_base2;
				}

				/*!
				Replaces the plane by p
				*/
				void update(const Plane_3& p) {
					m_base1 = p.base1() / std::sqrt(p.base1() * p.base1());
					m_base2 = p.base2() / std::sqrt(p.base2() * p.base2());
					m_normal = p.orthogonal_vector() / std::sqrt(p.orthogonal_vector() * p.orthogonal_vector());

					m_d = p.d();
				}

				// Returns a string with shape parameters.
				std::string info() const {
					std::stringstream sstr;
					sstr << "Type: plane (" << this->get_x(m_normal) << ", " << this->get_y(m_normal) << ", " << this->get_z(m_normal) << ")x - " << m_d << "= 0" << " #Pts: " << this->m_indices.size();
					return sstr.str();
				}

			protected:
				virtual void create_shape(const std::vector<std::size_t> &indices) {
					Point_3 p1 = this->point(indices[0]);
					Point_3 p2 = this->point(indices[1]);

					m_normal = this->cross_pdct(this->constr_vec(p2, p1), Vector_3(0, 0, 1));
					FT length = CGAL::sqrt(this->sqlen(m_normal));

					// Are the points almost singular?
					if (length < (FT)0.0001) {
						this->m_is_valid = false;
						return;
					}

					m_normal = this->scale(m_normal, (FT)1.0 / length);
					m_d = -(p1[0] * m_normal[0] + p1[1] * m_normal[1] + p1[2] * m_normal[2]);

					//check deviation of the 3 normal
					Vector_3 l_v = this->constr_vec();
					for (std::size_t i = 0; i < 2; i++) {
						l_v = this->normal(indices[i]);

						if (CGAL::abs(this->scalar_pdct(l_v, m_normal)) < this->m_normal_threshold * CGAL::sqrt(this->sqlen(l_v))) {
							this->m_is_valid = false;
							return;
						}

						m_point_on_primitive = p1;
						m_base1 = Vector_3(0, 0, 1);
						m_base2 = this->cross_pdct(m_base1, m_normal);
						m_base2 = this->scale(m_base2, ((FT)1.0 / CGAL::sqrt(this->sqlen(m_base2))));
					}

					this->m_is_valid = true;
				}

				virtual void parameters(const std::vector<std::size_t> &indices, std::vector<std::pair<FT, FT> > &parameterSpace, FT &, FT min[2], FT max[2]) const {
					// Transform first point before to initialize min/max
					Vector_3 p = this->constr_vec(m_point_on_primitive, this->point(indices[0]));
					FT u = this->scalar_pdct(p, m_base1);
					FT v = this->scalar_pdct(p, m_base2);
					parameterSpace[0] = std::pair<FT, FT>(u, v);
					min[0] = max[0] = u;
					min[1] = max[1] = v;

					for (std::size_t i = 1; i < indices.size(); i++) {
						p = this->constr_vec(m_point_on_primitive, this->point(indices[i]));
						u = this->scalar_pdct(p, m_base1);
						v = this->scalar_pdct(p, m_base2);
						min[0] = (std::min<FT>)(min[0], u);
						max[0] = (std::max<FT>)(max[0], u);
						min[1] = (std::min<FT>)(min[1], v);
						max[1] = (std::max<FT>)(max[1], v);
						parameterSpace[i] = std::pair<FT, FT>(u, v);
					}
				}

				// Computes squared Euclidean distance from a set of points.
				virtual void squared_distance(const std::vector<std::size_t> &indices, std::vector<FT> &dists) const {
					for (std::size_t i = 0; i < indices.size(); i++) {
						const FT d = this->scalar_pdct(this->constr_vec(m_point_on_primitive, this->point(indices[i])), m_normal);
						dists[i] = d * d;
					}
				}

				virtual void cos_to_normal(const std::vector<std::size_t> &indices,
					std::vector<FT> &angles) const {
					for (std::size_t i = 0; i < indices.size(); i++) {
						angles[i] = CGAL::abs(this->scalar_pdct(this->normal(indices[i]), m_normal));
					}
				}

				FT cos_to_normal(const Point_3 &, const Vector_3 &n) const{
					return CGAL::abs(this->scalar_pdct(n, m_normal));
				}

				virtual std::size_t minimum_sample_size() const {
					return 2;
				}

				virtual bool supports_connected_component() const {
					return true;
				}

			private:
				Point_3 m_point_on_primitive;
				Vector_3 m_base1, m_base2, m_normal;
				FT m_d;
			};

		}
	}

}