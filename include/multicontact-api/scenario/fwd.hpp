// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#ifndef __multicontact_api_scenario_fwd_hpp__
#define __multicontact_api_scenario_fwd_hpp__

#include "multicontact-api/geometry/fwd.hpp"
#include <pinocchio/spatial/fwd.hpp>

namespace multicontact_api
{
  namespace scenario
  {

    template<class T> struct traits {};

    template<typename _Scalar> struct ContactPatchTpl;
    typedef ContactPatchTpl<double> ContactPatch;

    template<typename _Scalar, int _dim> struct ContactPhaseTpl;
    typedef ContactPhaseTpl<double,4> ContactPhase4;

    template<typename _Scalar> struct ContactPhaseHumanoidTpl;
    typedef ContactPhaseHumanoidTpl<double> ContactPhaseHumanoid;

    template<class _ContactPhase> struct ContactSequenceTpl;
    typedef ContactSequenceTpl<ContactPhase4> ContactSequence4;
    typedef ContactSequenceTpl<ContactPhaseHumanoid> ContactSequenceHumanoid;


    template<class SOC> struct ContactConstraintSOC;
    typedef ContactConstraintSOC<geometry::SOC6d> ContactConstraintSOC6;

    template<typename Scalar> struct ContactModelPlanarTpl;
    typedef ContactModelPlanarTpl<double> ContactModelPlanar;

    template<typename Scalar> struct ContactConstraintPlanarTpl;
    typedef ContactConstraintPlanarTpl<double> ContactConstraintPlanar;

    enum HumanoidPhaseType
    {
      SINGLE_SUPPORT,
      DOUBLE_SUPPORT,
      TRIPLE_SUPPORT,
      QUADRUPLE_SUPPORT,
      HUMANOID_PHASE_UNDEFINED
    };

    enum ConicType
    {
      CONIC_SOWC,
      CONIC_DOUBLE_DESCRIPTION,
      CONIC_UNDEFINED
    };
  }
}

#endif // ifndef __multicontact_api_scenario_fwd_hpp__
