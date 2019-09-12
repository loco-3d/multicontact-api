// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_geometry_ellipsoid_hpp__
#define __multicontact_api_geometry_ellipsoid_hpp__

#include <Eigen/Dense>
#include <iostream>

#include "multicontact-api/geometry/fwd.hpp"

namespace multicontact_api {
namespace geometry {
template <typename _Scalar, int _dim, int _Options>
struct Ellipsoid {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  enum { dim = _dim };
  enum { Options = _Options };

  typedef Eigen::Matrix<Scalar, dim, dim, Options> Matrix;
  typedef Eigen::Matrix<Scalar, dim, 1, Options> Vector;

  Ellipsoid(const Matrix& A, const Vector& center) : m_A(A), m_center(center) {}

  Scalar lhsValue(const Vector& point) const { return (m_A * (point - m_center)).norm(); }

  const Matrix& A() const { return m_A; }
  Matrix& A() { return m_A; }
  const Vector& center() const { return m_center; }
  Vector& center() { return m_center; }

  void disp(std::ostream& os) const {
    os << "A:\n" << m_A << std::endl << "center: " << m_center.transpose() << std::endl;
  }

  friend std::ostream& operator<<(std::ostream& os, const Ellipsoid& E) {
    E.disp(os);
    return os;
  }

 protected:
  /// \brief
  Matrix m_A;

  /// \brief Center of the ellipsoid expressed in the global frame.
  Vector m_center;
};
}  // namespace geometry
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_geometry_ellipsoid_hpp__
