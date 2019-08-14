// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/scenario/contact-sequence.hpp"
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

BOOST_AUTO_TEST_CASE(contact_phase)
{
  ContactPhase4 cp, cp_test;
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
  BOOST_CHECK(cp == cp2);
  // test serialization
  cp.saveAsText("serialization_cp_test.test");
  cp_test.loadFromText("serialization_cp_test.test");
  remove("serialization_cp_test.test");
  BOOST_CHECK(cp == cp_test);
}

BOOST_AUTO_TEST_CASE(contact_sequence)
{
  ContactPhase4 cp;
  ContactSequence4 cs(1);
  ContactSequence4 cs_test(0);
  cs.m_contact_phases[0] = cp;
  // test serialization
  cs.saveAsText("serialization_cs_test.test");
  cs_test.loadFromText("serialization_cs_test.test");
  remove("serialization_cp_test.test");
  BOOST_CHECK(cs == cs_test);
}

BOOST_AUTO_TEST_SUITE_END()
