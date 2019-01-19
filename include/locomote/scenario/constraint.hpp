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
#ifndef __locomote_scenario_constraint_hpp__
#define __locomote_scenario_constraint_hpp__

#include "locomote/scenario/fwd.hpp"

namespace locomote
{
  namespace scenario
  {
    
    template<typename Derived>
    struct ConstraintBase
    {
      typedef Eigen::DenseIndex Index;
      typedef typename traits<Derived>::ArgumentType ArgumentType;
      typedef typename traits<Derived>::ReturnType ReturnType;
      
      const Derived & derived() const { return *static_cast<const Derived*> (this); }
      Derived & derived() { return *static_cast<Derived*> (this); }
      
      Index inputSize() const { return derived().inputSize(); }
      Index outputSize() const { return derived().outputSize(); }
      
      /// \returns the number of equality constraints among the inequalities.
      Index neq() const { return derived().neq(); }
      
      /// \returns the evaluation of the constraint at a given point
      ReturnType value(const ArgumentType & point) const
      { return derived().value(point); }
      
      ReturnType residu(const ArgumentType & point) const
      { return derived().residu(point); }
      
    };
    
  }
}

#endif // ifndef __locomote_scenario_constraint_hpp__
