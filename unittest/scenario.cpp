// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/scenario/contact-phase-humanoid.hpp"
#include "multicontact-api/scenario/contact-patch.hpp"

using namespace multicontact_api::scenario;

template <typename Scalar>
struct ATpl {
  typedef pinocchio::SE3Tpl<Scalar> SE3;

  explicit ATpl() : data() {}
  explicit ATpl(const ATpl& other) : data(other.data){};

  bool operator==(const ATpl& other) { return data == other.data; }

 protected:
  SE3 data;
};

typedef ATpl<double> Ad;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_model) {
  const double mu = 0.3;
  const double ZMP_radius = 0.01;

  ContactModelPlanar mp1(mu, ZMP_radius);
  ContactModelPlanar mp2(mp1);

  BOOST_CHECK(mp1 == mp2);
}

BOOST_AUTO_TEST_CASE(contact_patch) {
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

BOOST_AUTO_TEST_CASE(contact_phase) {
  ContactPhase4 cp;
  for(ContactPhase4::ContactPatchMap::iterator it = cp.contact_patches().begin();
      it !=  cp.contact_patches().end(); ++it)
  {
    it->second.contactModel().m_mu = 0.3;
    it->second.contactModel().m_ZMP_radius = 0.01;
    it->second.placement().setRandom();
    it->second.contactModelPlacement().setRandom();
    it->second.worldContactModelPlacement().setRandom();
  }
  ContactPhase4 cp2(cp);

  BOOST_CHECK(cp == cp);
  //  BOOST_CHECK(cp == cp2);
}
//
// BOOST_AUTO_TEST_CASE(contact_phase_humanoid)
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
