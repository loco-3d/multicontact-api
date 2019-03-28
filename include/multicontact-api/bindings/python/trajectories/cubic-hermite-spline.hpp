// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_trajectories_cubic_hermite_spline_hpp__
#define __multicontact_api_python_trajectories_cubic_hermite_spline_hpp__

#include <boost/python/tuple.hpp>
#include <string>

#include "multicontact-api/trajectories/cubic-hermite-spline.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/utils/printable.hpp"

namespace multicontact_api
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename CubicHermiteSpline>
    struct CubicHermiteSplinePythonVisitor
    : public boost::python::def_visitor< CubicHermiteSplinePythonVisitor<CubicHermiteSpline> >
    {
      typedef typename CubicHermiteSpline::Scalar Scalar;
      typedef typename CubicHermiteSpline::MatrixDx MatrixDx;
      typedef typename CubicHermiteSpline::VectorX VectorX;
      typedef typename CubicHermiteSpline::VectorD VectorD;
      typedef typename CubicHermiteSpline::Index Index;

      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<Index>(bp::arg("size"),"Init the hermite spline with a given size."))
          .def(bp::init<VectorX,MatrixDx,MatrixDx>(bp::args("abcissa", "points","derivatives"),"Init from a given set of points and derivatives, together with a vector of absicca."))
        .def(bp::init<CubicHermiteSpline>(bp::args("other"),"Copy contructor."))

        .add_property("absicca",
                      bp::make_function((const VectorX & (CubicHermiteSpline::*)(void)) &CubicHermiteSpline::absicca,
                                        bp::return_value_policy<bp::copy_const_reference>()),
                      &CubicHermiteSpline::setAbsicca,
                      "Abiscca of the spline.")
        .add_property("points",
                      bp::make_function((const MatrixDx & (CubicHermiteSpline::*)(void) const) &CubicHermiteSpline::points,
                                        bp::return_value_policy<bp::copy_const_reference>()),
                      &setPoints,
                      "Via points of the spline.")
        .add_property("derivatives",
                      bp::make_function((const MatrixDx & (CubicHermiteSpline::*)(void) const) &CubicHermiteSpline::derivatives,
                                        bp::return_value_policy<bp::copy_const_reference>()),
                      &setDerivatives,
                      "Derivative values of the via points of the spline.")

        .def("size",&CubicHermiteSpline::size,"Returns the number of points contained in the spline.")
        .def("resize",&CubicHermiteSpline::resize,bp::arg("size"),"Resize the spline.")
        .def("dimension",&CubicHermiteSpline::dimension,"Returns the dimension of the spline.")
        .def("numIntervals",&CubicHermiteSpline::numIntervals,"Returns the number of intervals contained in the spline.")

        .def("eval",&eval,bp::arg("t"),"Eval the spline at a given abscicca t.")
        .def(bp::self+bp::self)
        .def(bp::self-bp::self)
        .def(bp::self+=bp::self)
        .def(bp::self-=bp::self)
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy",&copy,"Returns a copy of *this.")

        .def("Constant",&CubicHermiteSpline::Constant,bp::arg("point"),"Returns a constant spline passing through the input point.")
        .staticmethod("Constant")
        ;
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Cubic Hermite Spline";
        bp::class_<CubicHermiteSpline>(class_name.c_str(),
                                       doc.c_str(),
                                       bp::no_init)
        .def(CubicHermiteSplinePythonVisitor<CubicHermiteSpline>())
        .def(SerializableVisitor<CubicHermiteSpline>())
//        .def(PrintableVisitor<CubicHermiteSpline>())
        ;

      }

    protected:

      static void setPoints(CubicHermiteSpline & self, const MatrixDx & points)
      { self.points() = points; }
      static void setDerivatives(CubicHermiteSpline & self, const MatrixDx & derivatives)
      { self.derivatives() = derivatives; }

      static bp::tuple eval(const CubicHermiteSpline & self, const Scalar t)
      {
        VectorD point(VectorD::Zero(self.dimension())), derivative(VectorD::Zero(self.dimension()));
        self.eval(t,point,derivative);

        return bp::make_tuple(point,derivative);
      }

      static CubicHermiteSpline copy(const CubicHermiteSpline & self) { return CubicHermiteSpline(self); }

    };

    void exposeSplineAlgos()
    {
      bp::def("createHermiteSplineAtAbsicca",
              &multicontact_api::trajectories::createHermiteSplineAtAbsicca<double, 3>,
              bp::args("Spline", "New Abscissa"),
              "create and return a new spline at the points of the new abscissa."
              "The size of the new abscissa needs to be higher so that there is no information loss.");
    }

  }
}


#endif // ifndef __multicontact_api_python_trajectories_cubic_hermite_spline_hpp__
