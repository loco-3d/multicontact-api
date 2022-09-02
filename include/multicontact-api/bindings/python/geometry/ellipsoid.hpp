// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_geometry_ellipsoid_hpp__
#define __multicontact_api_python_geometry_ellipsoid_hpp__

#include <pinocchio/fwd.hpp>

#include "multicontact-api/bindings/python/fwd.hpp"
#include "multicontact-api/geometry/ellipsoid.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename Ellipsoid>
struct EllipsoidPythonVisitor
    : public boost::python::def_visitor<EllipsoidPythonVisitor<Ellipsoid> > {
  typedef typename Ellipsoid::Matrix Matrix;
  typedef typename Ellipsoid::Vector Vector;

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<Matrix, Vector>((bp::arg("A"), bp::arg("center"))))
        .def("__str__", &toString)
        .def("lhsValue", &Ellipsoid::lhsValue, bp::arg("point"),
             "Returns the value of norm(A*(x-c)).")
        .add_property("center", &get_center, &set_center,
                      "Accessor to the center property.")
        .add_property("A", &get_A, &set_A, "Accessor to the A property.");
  }

  static void set_center(Ellipsoid& e, const Vector& center) {
    e.center() = center;
  }
  static Vector get_center(const Ellipsoid& e) { return e.center(); }

  static void set_A(Ellipsoid& e, const Matrix& A) { e.A() = A; }
  static Matrix get_A(const Ellipsoid& e) { return e.A(); }

  static void expose(const std::string& class_name) {
    std::string doc = "Ellipsoid of dimension " + Ellipsoid::dim;
    doc += " defined by its matrix A and its center.";
    bp::class_<Ellipsoid>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(EllipsoidPythonVisitor<Ellipsoid>());
  }

 protected:
  static std::string toString(const Ellipsoid& e) {
    std::ostringstream s;
    s << e;
    return s.str();
  }
};

}  // namespace python
}  // namespace multicontact_api

#endif  // ifnef __multicontact_api_python_geometry_ellipsoid_hpp__
