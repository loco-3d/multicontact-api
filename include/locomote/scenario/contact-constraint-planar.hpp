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
#ifndef __locomote_scenario_contact_constraint_planar_hpp__
#define __locomote_scenario_contact_constraint_planar_hpp__

#include "locomote/scenario/contact-constraint.hpp"
#include "locomote/scenario/contact-model-planar.hpp"

#include <pinocchio/spatial/force.hpp>

namespace locomote
{
  namespace scenario
  {
    template<typename _Scalar>
    struct traits< ContactConstraintPlanarTpl<_Scalar> >
    {
      typedef _Scalar Scalar;
      typedef se3::ForceTpl<Scalar,0> Force;
      typedef ContactModelPlanarTpl<Scalar> ContactModel;
      
      enum
      {
        dim_in = 6,
        dim_out = 2
      };
      
      typedef Force ArgumentType;
      typedef Eigen::Matrix<Scalar,dim_out,1> ReturnType;
    };
    
    template<typename _Scalar>
    struct ContactConstraintPlanarTpl
    : public ContactConstraintBase< ContactConstraintPlanarTpl<_Scalar> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef _Scalar Scalar;
      typedef ContactConstraintBase< ContactConstraintPlanarTpl<Scalar> > Base;
      typedef ContactModelPlanarTpl<Scalar> ContactModel;
      typedef Eigen::DenseIndex Index;
      
      typedef typename traits<ContactConstraintPlanarTpl>::ArgumentType ArgumentType;
      typedef typename traits<ContactConstraintPlanarTpl>::ReturnType ReturnType;
      
      using Base::contactModel;
      
      /// \brief Default constructor
      ContactConstraintPlanarTpl()
      : Base()
      {}
      
      ContactConstraintPlanarTpl(const ContactModel & contact_model)
      : Base(contact_model)
      {}
      
      static Index inputSize() { return traits< ContactConstraintPlanarTpl >::dim_in; }
      static Index outputSize() { return traits< ContactConstraintPlanarTpl >::dim_out; }
      static Index neq() { return 0; }
      
      ReturnType value(const ArgumentType & f) const
      {
        typedef Eigen::Matrix<Scalar,2,1> Vector2;
        
        const ContactModel & contact_model = contactModel();
        const Scalar & mu = contact_model.m_mu;
        const Scalar & ZMP_radius = contact_model.m_ZMP_radius;
        
        const Scalar & fx = f.linear()[0];
        const Scalar & fy = f.linear()[1];
        const Scalar & fz = f.linear()[2];
        const Scalar & tx = f.angular()[0];
        const Scalar & ty = f.angular()[1];
        const Scalar & tz = f.angular()[2];
        
        const Scalar mu_fz = mu*fz;
        const Scalar ZMP_radius_fx = fx*ZMP_radius;
        const Scalar ZMP_radius_fy = fy*ZMP_radius;
        const Scalar ZMP_radius_fz = fz*ZMP_radius;
        ReturnType res;
        res[0] = mu_fz*mu_fz - f.linear().template head<2>().squaredNorm();
        res[1] = ZMP_radius_fz*ZMP_radius_fz - f.angular().template head<2>().squaredNorm();
        
        const Scalar & tmin = -4.*mu*ZMP_radius_fz
        + std::fabs(ZMP_radius * fx - mu*tx)
        + std::fabs(ZMP_radius * fy - mu*ty);
        
        const Scalar & tmax = 4.*mu*ZMP_radius_fz
        - std::fabs(ZMP_radius * fx + mu*tx)
        - std::fabs(ZMP_radius * fy + mu*ty);
        
        //res[2] = tz - tmin;
        //res[3] = -tz + tmax;
        
        return res;
      }
      
      ReturnType residu(const ArgumentType & f) const
      { return value(f); }
      
      
    };
  }
}

#endif // ifndef __locomote_scenario_contact_constraint_planar_hpp__
