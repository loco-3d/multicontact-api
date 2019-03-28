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

#ifndef __locomote_python_scenario_contact_model_planar_hpp__
#define __locomote_python_scenario_contact_model_planar_hpp__

#include <string>

#include "locomote/scenario/contact-model-planar.hpp"
#include "locomote/bindings/python/serialization/archive.hpp"
#include "locomote/bindings/python/utils/printable.hpp"


namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename ContactModelPlanar>
    struct ContactModelPlanarPythonVisitor
    : public boost::python::def_visitor< ContactModelPlanarPythonVisitor<ContactModelPlanar> >
    {
      typedef typename ContactModelPlanar::Scalar Scalar;
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>())
        .def(bp::init<Scalar,Scalar>(bp::args("mu","ZMP_radius")))
        .def(bp::init<ContactModelPlanar>(bp::args("other"),"Copy contructor."))
        .def_readwrite("mu",&ContactModelPlanar::m_mu,"Friction coefficient.")
        .def_readwrite("ZMP_radius",&ContactModelPlanar::m_ZMP_radius,"Radius of the ZMP region.")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy",&copy,"Returns a copy of *this.")
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Contact Model Planar";
        bp::class_<ContactModelPlanar>(class_name.c_str(),
                                       doc.c_str(),
                                       bp::no_init)
        .def(ContactModelPlanarPythonVisitor<ContactModelPlanar>())
        .def(SerializableVisitor<ContactModelPlanar>())
        .def(PrintableVisitor<ContactModelPlanar>())
        ;
        
      }
      
    private:
      
      static ContactModelPlanar copy(const ContactModelPlanar & self)
      { return ContactModelPlanar(self); }
  
    };
  }
}


#endif // ifndef __locomote_python_scenario_contact_model_planar_hpp__
