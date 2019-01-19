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
#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "locomote/scenario/contact-phase.hpp"
#include "locomote/scenario/contact-phase-humanoid.hpp"
#include "locomote/scenario/contact-patch.hpp"

using namespace locomote::scenario;

template<typename Scalar>
struct ATpl
{
  typedef se3::SE3Tpl<Scalar> SE3;
  
  explicit ATpl() : data() {}
  explicit ATpl(const ATpl & other) : data(other.data) {};
  
  bool operator==(const ATpl & other) { return data == other.data; }
  
protected:
  
  SE3 data;
};

typedef ATpl<double> Ad;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)


BOOST_AUTO_TEST_CASE(contact_model)
{
  const double mu = 0.3;
  const double ZMP_radius = 0.01;
  
  ContactModelPlanar mp1(mu,ZMP_radius);
  ContactModelPlanar mp2(mp1);
  
  BOOST_CHECK(mp1 == mp2);
}

BOOST_AUTO_TEST_CASE(contact_patch)
{
  ContactPatch cp;
  cp.contactModel().m_mu = 0.3;
  cp.contactModel().m_ZMP_radius = 0.01;
  cp.placement().setRandom();
  cp.contactModelPlacement().setRandom();
  cp.worldContactModelPlacement().setRandom();
  ContactPatch cp2(cp);
  ContactPatch cp3 = cp2;
  
  BOOST_CHECK(cp == cp);
  BOOST_CHECK(cp == cp2);
  
  Ad a1;
  Ad a2(a1);
  
}

BOOST_AUTO_TEST_CASE(contact_phase)
{
  ContactPhase4 cp;
  for(ContactPhase4::ContactPatchArray::iterator it = cp.contact_patches().begin();
      it !=  cp.contact_patches().end(); ++it)
  {
    it->contactModel().m_mu = 0.3;
    it->contactModel().m_ZMP_radius = 0.01;
    it->placement().setRandom();
    it->contactModelPlacement().setRandom();
    it->worldContactModelPlacement().setRandom();
  }
  ContactPhase4 cp2(cp);
  
  BOOST_CHECK(cp == cp);
//  BOOST_CHECK(cp == cp2);
}
//
//BOOST_AUTO_TEST_CASE(contact_phase_humanoid)
//{
//  ContactPhaseHumanoid cp;
//  for(ContactPhaseHumanoid::ContactPatchArray::iterator it = cp.contact_patches().begin();
//      it !=  cp.contact_patches().end(); ++it)
//  {
//    it->contactModel().m_mu = 0.3;
//    it->contactModel().m_ZMP_radius = 0.01;
//  }
//  ContactPhaseHumanoid cp2(cp);
//  
//  BOOST_CHECK(cp == cp);
//  BOOST_CHECK(cp == cp2);
//  
//  cp.m_reference_configurations.push_back(Eigen::VectorXd::Zero(10));
//  
////  BOOST_CHECK(cp != cp2);
//}

BOOST_AUTO_TEST_SUITE_END()
