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

#ifndef __locomote_python_geometry_second_order_cone_hpp__
#define __locomote_python_geometry_second_order_cone_hpp__

#include <eigenpy/eigenpy.hpp>

#include "locomote/geometry/second-order-cone.hpp"
#include "locomote/bindings/python/serialization/archive.hpp"

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename SOC>
    struct SOCPythonVisitor
    : public boost::python::def_visitor< SOCPythonVisitor<SOC> >
    {
      typedef typename SOC::MatrixD MatrixD;
      typedef typename SOC::VectorD VectorD;
      typedef typename SOC::Scalar Scalar;
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<MatrixD,VectorD>
             ((bp::arg("Q"),bp::arg("direction"))))
        .def("__str__",&toString)
        .def("lhsValue",&SOC::lhsValue,bp::arg("vector"),"Returns the lhs value of the conic inequality.")
        .def("rhsValue",&SOC::rhsValue,bp::arg("vector"),"Returns the rhs value of the conic inequality.")
        .def("check",(bool (SOC::*)(const VectorD &) const)&SOC::check,bp::arg("vector"),"Checks if the vector given in argument belongs to the conic constraint.")
        .def("check",(bool (SOC::*)(const VectorD &, const Scalar) const)&SOC::check,bp::args ("vector","factor"),"Checks if the vector given in argument belongs to the conic constraint with a given reduction factor.")
        .add_property("direction",&get_direction,&SOC::setDirection,"Accessor to the direction property.")
        .add_property("Q",&get_Q,&SOC::setQ,"Accessor to the Q property.")
        .def("isApprox",(bool (SOC::*)(const SOC &, const Scalar &) const)&SOC::isApprox,
             bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision determined by prec.")
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("RegularCone",&SOC::RegularCone,
             bp::args("mu","direction"),"Creates a regular cone from a given friction coefficient and a direction.")
        .staticmethod("RegularCone")
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "SOC of dimension " + SOC::dim;
        doc += " defined by its direction and its quadratic norm.";
        bp::class_<SOC>(class_name.c_str(),
                        doc.c_str(),
                        bp::no_init)
        .def(SOCPythonVisitor<SOC>())
        .def(SerializableVisitor<SOC>())
        ;
        
        // Expose related matrix types
        eigenpy::enableEigenPySpecific<MatrixD,MatrixD>();
        eigenpy::enableEigenPySpecific<VectorD,VectorD>();
      }
      
    protected:
      static std::string toString(const SOC & c) { std::ostringstream s; s << c; return s.str(); }
      
      static VectorD get_direction(const SOC & c) { return c.direction(); }
      static MatrixD get_Q(const SOC & c) { return c.Q(); }
    };
    
  }
}

#endif // ifnef __locomote_python_geometry_second_order_cone_hpp__
