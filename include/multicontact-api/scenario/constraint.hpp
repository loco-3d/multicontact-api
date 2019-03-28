// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_constraint_hpp__
#define __multicontact_api_scenario_constraint_hpp__

#include "multicontact-api/scenario/fwd.hpp"

namespace multicontact_api
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

#endif // ifndef __multicontact_api_scenario_constraint_hpp__
