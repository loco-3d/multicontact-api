// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_geometry_second_order_cone_hpp__
#define __multicontact_api_python_geometry_second_order_cone_hpp__

#include <eigenpy/eigenpy.hpp>

#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/geometry/second-order-cone.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename SOC>
struct SOCPythonVisitor
    : public boost::python::def_visitor<SOCPythonVisitor<SOC> > {
  typedef typename SOC::MatrixD MatrixD;
  typedef typename SOC::VectorD VectorD;
  typedef typename SOC::Scalar Scalar;

  template <class PyClass>
  void visit(PyClass &cl) const {
    cl.def(bp::init<>("Default constructor."))
        .def(bp::init<MatrixD, VectorD>((bp::arg("Q"), bp::arg("direction"))))
        .def("__str__", &toString)
        .def("lhsValue", &SOC::lhsValue, bp::arg("vector"),
             "Returns the lhs value of the conic inequality.")
        .def("rhsValue", &SOC::rhsValue, bp::arg("vector"),
             "Returns the rhs value of the conic inequality.")
        .def("check", (bool(SOC::*)(const VectorD &) const) & SOC::check,
             bp::arg("vector"),
             "Checks if the vector given in argument belongs to the conic "
             "constraint.")
        .def("check",
             (bool(SOC::*)(const VectorD &, const Scalar) const) & SOC::check,
             bp::args("vector", "factor"),
             "Checks if the vector given in argument belongs to the conic "
             "constraint with a given reduction factor.")
        .add_property("direction", &get_direction, &SOC::setDirection,
                      "Accessor to the direction property.")
        .add_property("Q", &get_Q, &SOC::setQ, "Accessor to the Q property.")
        .def("isApprox",
             (bool(SOC::*)(const SOC &, const Scalar &) const) & SOC::isApprox,
             bp::args("other", "prec"),
             "Returns true if *this is approximately equal to other, within "
             "the precision determined by prec.")
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)

        .def("RegularCone", &SOC::RegularCone, bp::args("mu", "direction"),
             "Creates a regular cone from a given friction coefficient and a "
             "direction.")
        .staticmethod("RegularCone");
  }

  static void expose(const std::string &class_name) {
    std::string doc = "SOC of dimension " + SOC::dim;
    doc += " defined by its direction and its quadratic norm.";
    bp::class_<SOC>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(SOCPythonVisitor<SOC>())
        .def(SerializableVisitor<SOC>());

    // Expose related matrix types
    ENABLE_SPECIFIC_MATRIX_TYPE(MatrixD);
    ENABLE_SPECIFIC_MATRIX_TYPE(VectorD);
  }

 protected:
  static std::string toString(const SOC &c) {
    std::ostringstream s;
    s << c;
    return s.str();
  }

  static VectorD get_direction(const SOC &c) { return c.direction(); }
  static MatrixD get_Q(const SOC &c) { return c.Q(); }
};

}  // namespace python
}  // namespace multicontact_api

#endif  // ifnef __multicontact_api_python_geometry_second_order_cone_hpp__
