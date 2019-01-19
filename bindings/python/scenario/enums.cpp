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
#include "locomote/scenario/fwd.hpp"

#include "locomote/bindings/python/scenario/expose-scenario.hpp"
#include "locomote/bindings/python/scenario/enums.hpp"

#include <boost/python/enum.hpp>

namespace locomote
{
  namespace python
  {
    namespace bp = boost::python;
    
    using namespace locomote::scenario;
    
    void exposeEnumHumanoidPhaseType()
    {
      bp::enum_<HumanoidPhaseType>("HumanoidPhaseType")
      .value("SINGLE_SUPPORT",SINGLE_SUPPORT)
      .value("DOUBLE_SUPPORT",DOUBLE_SUPPORT)
      .value("TRIPLE_SUPPORT",TRIPLE_SUPPORT)
      .value("QUADRUPLE_SUPPORT",QUADRUPLE_SUPPORT)
      .value("HUMANOID_PHASE_UNDEFINED",HUMANOID_PHASE_UNDEFINED)
      ;
    }
    
    void exposeEnumConicType()
    {
      bp::enum_<ConicType>("ConicType")
      .value("CONIC_SOWC",CONIC_SOWC)
      .value("CONIC_DOUBLE_DESCRIPTION",CONIC_DOUBLE_DESCRIPTION)
      .value("CONIC_UNDEFINED",CONIC_UNDEFINED)
      ;
    }
    
    void exposeScenarioEnums()
    {
      exposeEnumHumanoidPhaseType();
      exposeEnumConicType();
    }
  }
}
