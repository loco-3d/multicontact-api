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
#ifndef __locomote_scenario_contact_model_planar_hpp__
#define __locomote_scenario_contact_model_planar_hpp__

#include "locomote/scenario/fwd.hpp"
#include "locomote/serialization/archive.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace locomote
{
  namespace scenario
  {
    
    template<typename _Scalar>
    struct ContactModelPlanarTpl : public serialization::Serializable< ContactModelPlanarTpl<_Scalar> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef _Scalar Scalar;
      
      /// \brief Default constructor.
      ContactModelPlanarTpl()
      : m_mu(-1.)
      , m_ZMP_radius(-1.)
      {}
      
      /// \brief Default constructor.
      ContactModelPlanarTpl(const Scalar mu, const Scalar ZMP_radius)
      : m_mu(mu)
      , m_ZMP_radius(ZMP_radius)
      {}
      
      
      /// \brief Copy constructor
      template<typename S2>
      explicit ContactModelPlanarTpl(const ContactModelPlanarTpl<S2> & other)
      : m_mu(other.mu)
      , m_ZMP_radius(other.ZMP_radius)
      {}
      
      template<typename S2>
      bool operator==(const ContactModelPlanarTpl<S2> & other) const
      {
        return m_mu == other.m_mu
        && m_ZMP_radius == other.m_ZMP_radius;
      }
      
      template<typename S2>
      bool operator!=(const ContactModelPlanarTpl<S2> & other) const
      { return !(*this != other); }
      
      void disp(std::ostream & os) const
      {
        os
        << "mu: " << m_mu << std::endl
        << "ZMP radius: " << m_ZMP_radius << std::endl
        ;
      }
      
      template<typename S2>
      friend std::ostream & operator <<(std::ostream & os, const ContactModelPlanarTpl<S2> & cp)
      {
        cp.disp(os);
        return os;
      }
      
      /// \brief Friction coefficient.
      Scalar m_mu;
      
      /// \brief ZMP radius.
      Scalar m_ZMP_radius;
      
    private:
      
      // Serialization of the class
      friend class boost::serialization::access;
      
      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        ar & boost::serialization::make_nvp("mu",m_mu);
        ar & boost::serialization::make_nvp("ZMP_radius",m_ZMP_radius);
      }
      
      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        ar >> boost::serialization::make_nvp("mu",m_mu);
        ar >> boost::serialization::make_nvp("ZMP_radius",m_ZMP_radius);
      }
      
      BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
  }
}

#endif // ifndef __locomote_scenario_contact_model_planar_hpp__
