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

#ifndef __locomote_python_geometry_ellipsoid_hpp__
#define __locomote_python_geometry_ellipsoid_hpp__

#include <boost/python.hpp>

#include "locomote/geometry/ellipsoid.hpp"

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename Ellipsoid>
    struct EllipsoidPythonVisitor
    : public boost::python::def_visitor< EllipsoidPythonVisitor<Ellipsoid> >
    {
      typedef typename Ellipsoid::Matrix Matrix;
      typedef typename Ellipsoid::Vector Vector;
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<Matrix,Vector>
             ((bp::arg("A"),bp::arg("center"))))
        .def("__str__",&toString)
        .def("lhsValue",&Ellipsoid::lhsValue,bp::arg("point"),"Returns the value of norm(A*(x-c)).")
        .add_property("center",&get_center,&set_center,"Accessor to the center property.")
        .add_property("A",&get_A,&set_A,"Accessor to the A property.")
        ;
      }
      
      static void set_center(Ellipsoid & e, const Vector & center) { e.center() = center; }
      static Vector get_center(const Ellipsoid & e) { return e.center(); }
      
      static void set_A(Ellipsoid & e, const Matrix & A) { e.A() = A; }
      static Matrix get_A(const Ellipsoid & e) { return e.A(); }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Ellipsoid of dimension " + Ellipsoid::dim;
        doc += " defined by its matrix A and its center.";
        bp::class_<Ellipsoid>(class_name.c_str(),
                              doc.c_str(),
                              bp::no_init)
        .def(EllipsoidPythonVisitor<Ellipsoid>())
        ;
        
      }
      
    protected:
      static std::string toString(const Ellipsoid & e) { std::ostringstream s; s << e; return s.str(); }
    };
    
  }
}

#endif // ifnef __locomote_python_geometry_ellipsoid_hpp__
