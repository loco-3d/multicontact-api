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
#ifndef __locomote_python_container_reference_wrapper_hpp__
#define __locomote_python_container_reference_wrapper_hpp__

#include <boost/python.hpp>
#include <boost/ref.hpp>
#include <boost/python/to_python_indirect.hpp>
#include <typeinfo>

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename reference_wrapper_type>
    struct reference_wrapper_converter
    {
      typedef reference_wrapper_type ref_type;
      typedef typename reference_wrapper_type::type type;
      
      static PyObject* convert(ref_type const& ref)
      {
        ref_type* const p = &const_cast<ref_type&>(ref);
        if(p == 0)
          return bp::detail::none();
        return bp::detail::make_reference_holder::execute(&p->get());
      }
      
      static void expose()
      {
        bp::to_python_converter<ref_type,reference_wrapper_converter>();
      }
    };
  }
}

#endif // ifndef __locomote_python_container_reference_wrapper_hpp__
