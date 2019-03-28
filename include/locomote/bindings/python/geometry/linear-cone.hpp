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

#ifndef __locomote_python_geometry_linear_cone_hpp__
#define __locomote_python_geometry_linear_cone_hpp__

#include <eigenpy/eigenpy.hpp>

#include "locomote/geometry/linear-cone.hpp"
#include "locomote/bindings/python/serialization/archive.hpp"

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename LC>
    struct LCPythonVisitor
    : public boost::python::def_visitor< LCPythonVisitor<LC> >
    {
      typedef bp::class_<LC> PyClass;
      typedef LC Type;
      
      typedef typename LC::MatrixDx MatrixDx;
      typedef typename LC::VectorD VectorD;
      typedef typename LC::Scalar Scalar;
      typedef typename LC::Index Index;
      
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<MatrixDx>((bp::arg("rays"),"Init from a set of rays.")))
        .def(bp::init<Index>(bp::args("size"),"Init with a given size."))
        .def(bp::init<LC>(bp::args("other"),"Copy constructor."))
        
        .add_property("size",&LC::size,"Returns the size of the set of rays.")
        .add_static_property("dim",&dim,"Dimension of the linear cone.")
        
        .add_property("rays",&getRays,&setRays,"Matrix of rays of the linear cone.")
        .def("__str__",&toString)
        .def("isApprox",(bool (LC::*)(const LC &, const Scalar &) const)&LC::isApprox,
             bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision determined by prec.")
        .def("stack",&LC::template stack<MatrixDx>,bp::args("rays"),"Stack new rays to the set of rays.")
        .def("stack",&LC::template stack<Scalar,LC::Options>,bp::args("cone"),"Stack the rays of one to the set of rays.")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static PyClass & expose(const std::string & class_name, std::string doc = "")
      {
        if(doc.empty())
        {
          doc = "Linear Cone of dimension " + LC::dim;
          doc += " defined by its rays.";
        }
        
        static PyClass cl_ = PyClass(class_name.c_str(),doc.c_str(),bp::no_init);
        cl_
        .def(LCPythonVisitor<LC>())
        .def(SerializableVisitor<LC>())
        ;
        
        // Expose related matrix types
        eigenpy::enableEigenPySpecific<MatrixDx,MatrixDx>();
        eigenpy::enableEigenPySpecific<VectorD,VectorD>();
        
        return cl_;
      }
      
    protected:
      static std::string toString(const LC & c) { std::ostringstream s; s << c; return s.str(); }
      
      static MatrixDx getRays(const LC & self) { return self.rays(); }
      static void setRays(LC & self, const MatrixDx & rays) { self.rays() = rays; }
      
      static int dim() { return LC::dim; }
    };
    
    
    template<typename ForceCone>
    struct ForceConePythonVisitor : public boost::python::def_visitor< ForceConePythonVisitor<ForceCone> >
    {
     
      typedef typename ForceCone::Scalar Scalar;
      typedef typename ForceCone::Vector3 Vector3;
      typedef typename ForceCone::VectorD VectorD;
      typedef typename ForceCone::Matrix3x Matrix3x;
      typedef typename ForceCone::Index Index;
      typedef typename ForceCone::WrenchCone WrenchCone;
      
      template<class _PyClass>
      void visit(_PyClass & cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<Matrix3x>((bp::arg("rays"),"Init from a matrix of rays.")))
        .def(bp::init<Index>(bp::args("size"),"Init with a given size."))
        .def(bp::init<ForceCone>(bp::args("other"),"Copy constructor."))
        
        .def("SE3ActOn",&ForceCone::SE3ActOn,bp::args("M"),"Returns the action of SE3 on *this, i.e. a WrenchCone.")
        .def("toWrenchCone",&toWrenchCone,"Returns *this as a WrenchCone.")
        
        .def("RegularCone",(ForceCone (*)(const Scalar, const VectorD &, const int)) &ForceCone::RegularCone,bp::args("mu","direction","num rays"),"Generates a regular linear cone from a given number of rays, a main direction and a friction coefficient.")
        .def("RegularCone",(ForceCone (*)(const Scalar, const VectorD &, const int, const Scalar)) &ForceCone::RegularCone,bp::args("mu","direction","num rays","angle offset"),"Generates a regular linear cone from a given number of rays, a main direction and a friction coefficient, with an offset on the orientation.")
        .staticmethod("RegularCone")
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Force Cone of dimension 3";
        doc += " defined by its rays.";
        
        LCPythonVisitor<typename ForceCone::Base>::expose("LinearCone3");
        
        bp::class_<ForceCone,bp::bases<typename ForceCone::Base> > (class_name.c_str(),doc.c_str(),bp::no_init)
        .def(ForceConePythonVisitor<ForceCone>())
        ;
      }
      
      static WrenchCone toWrenchCone(const ForceCone & self) { return (WrenchCone)(self); }
      
    };
    
    template<typename WrenchCone>
    struct WrenchConePythonVisitor : public boost::python::def_visitor< WrenchConePythonVisitor<WrenchCone> >
    {
      
      typedef typename WrenchCone::Matrix3x Matrix3x;
      typedef typename WrenchCone::Matrix6x Matrix6x;
      typedef typename WrenchCone::Index Index;
      
      
      template<class _PyClass>
      void visit(_PyClass & cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<Matrix6x>((bp::arg("rays"),"Init from a matrix of rays.")))
        .def(bp::init<Index>(bp::args("size"),"Init with a given size."))
        .def(bp::init<WrenchCone>(bp::args("other"),"Copy constructor."))
        
        .def("SE3ActOn",&WrenchCone::SE3ActOn,bp::args("M"),"Returns the action of SE3 on *this, i.e. a WrenchCone.")
        .def("linear",&getLinear,"Returns the linear block of *this.")
        .def("angular",&getAngular,"Returns the angular block of *this.")
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Linear Wrench Cone";
        
        LCPythonVisitor<typename WrenchCone::Base>::expose("LinearCone6");
        
        bp::class_<WrenchCone,bp::bases<typename WrenchCone::Base> > (class_name.c_str(),doc.c_str(),bp::no_init)
        .def(WrenchConePythonVisitor<WrenchCone>())
        ;
      }
      
    protected:
      static Matrix3x getLinear(const WrenchCone & self) { return self.linear(); }
      static Matrix3x getAngular(const WrenchCone & self) { return self.angular(); }
      
    };
    
  }
}

#endif // ifnef __locomote_python_geometry_linear_cone_hpp__
