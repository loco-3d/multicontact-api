// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
// Simplified BSD license :
//Redistribution and use in source and binary forms, with or without modification,
//are permitted provided that the following conditions are met:

//1. Redistributions of source code must retain the above copyright notice,
//this list of conditions and the following disclaimer.

//2. Redistributions in binary form must reproduce the above copyright notice,
//this list of conditions and the following disclaimer in the documentation
//and/or other materials provided with the distribution.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
//OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef __locomote_python_trajectories_cubic_hermite_spline_hpp__
#define __locomote_python_trajectories_cubic_hermite_spline_hpp__

#include <boost/python/tuple.hpp>
#include <string>

#include "locomote/trajectories/cubic-hermite-spline.hpp"
#include "locomote/bindings/python/serialization/archive.hpp"
#include "locomote/bindings/python/utils/printable.hpp"

namespace locomote
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

    private:
      static CubicHermiteSpline __add__(const CubicHermiteSpline & self,
                                        const CubicHermiteSpline & other)
      { return self+other; }
      static CubicHermiteSpline __sub__(const CubicHermiteSpline & self,
                                        const CubicHermiteSpline & other)
      { return self-other; }

    };
  }
}


#endif // ifndef __locomote_python_trajectories_cubic_hermite_spline_hpp__
