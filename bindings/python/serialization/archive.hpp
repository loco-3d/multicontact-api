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

#ifndef __locomote_python_serialization_archive_hpp__
#define __locomote_python_serialization_archive_hpp__

#include <boost/python.hpp>

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename Derived>
    struct SerializableVisitor
    : public boost::python::def_visitor< SerializableVisitor<Derived> >
    {
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def("saveAsText",&Derived::saveAsText,bp::args("filename"),"Saves *this inside a text file.")
        .def("loadFromText",&Derived::loadFromText,bp::args("filename"),"Loads *this from a text file.")
        .def("saveAsXML",&Derived::saveAsXML,bp::args("filename","tag_name"),"Saves *this inside a XML file.")
        .def("loadFromXML",&Derived::loadFromXML,bp::args("filename","tag_name"),"Loads *this from a XML file.")
        .def("saveAsBinary",&Derived::saveAsBinary,bp::args("filename"),"Saves *this inside a binary file.")
        .def("loadFromBinary",&Derived::loadFromBinary,bp::args("filename"),"Loads *this from a binary file.")
        ;
      }

    };
  }
}

#endif // ifndef __locomote_python_serialization_archive_hpp__
