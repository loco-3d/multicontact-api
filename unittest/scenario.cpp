// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/scenario/contact-model-planar.hpp"
#include "multicontact-api/scenario/contact-patch.hpp"
//#include "multicontact-api/scenario/contact-phase.hpp"
//#include "multicontact-api/scenario/contact-sequence.hpp"

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
typedef pinocchio::SE3Tpl<double> SE3;


BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_model) {
  const double mu = 0.3;
  const double ZMP_radius = 0.01;

  ContactModelPlanar mp1(mu, ZMP_radius);
  ContactModelPlanar mp2(mp1);

  BOOST_CHECK(mp1 == mp2);
}

BOOST_AUTO_TEST_CASE(contact_patch) {
  // check default constructor :
  ContactPatch cp;
  BOOST_CHECK(cp.placement() == SE3::Identity());
  BOOST_CHECK(cp.friction() == -1.);
  SE3 p = SE3::Identity();
  p.setRandom();
  cp.placement() = p;
  BOOST_CHECK(cp.placement() == p);
  cp.friction() = 0.7;
  BOOST_CHECK(cp.friction() == 0.7);

  // constructor with placement :
  p.setRandom();
  ContactPatch cp1(p);
  BOOST_CHECK(cp1.placement() == p);
  BOOST_CHECK(cp1.friction() == -1.);

  // constructor with placement and friction
  p.setRandom();
  ContactPatch cp2(p,0.9);
  BOOST_CHECK(cp2.placement() == p);
  BOOST_CHECK(cp2.friction() == 0.9);

  // check comparison operator
  BOOST_CHECK(cp1 != cp2);
  ContactPatch cp3(p,0.9);
  BOOST_CHECK(cp3 == cp2);
  cp2.friction() = 0.1;
  BOOST_CHECK(cp3 != cp2);

  // copy constructor
  ContactPatch cp4(cp3);
  BOOST_CHECK(cp4 == cp3);
  BOOST_CHECK(cp4.placement() == p);
  BOOST_CHECK(cp4.friction() == 0.9);
  cp4.placement() = SE3::Identity();
  BOOST_CHECK(cp3 != cp4);

  // serialization :
  std::string fileName("fileTest");
  cp3.saveAsText(fileName);
  ContactPatch cp_from_text;
  cp_from_text.loadFromText(fileName);
  BOOST_CHECK(cp3 == cp_from_text);

  cp3.saveAsXML(fileName,"ContactPatch");
  ContactPatch cp_from_xml;
  cp_from_xml.loadFromXML(fileName,"ContactPatch");
  BOOST_CHECK(cp3 == cp_from_xml);

  cp3.saveAsBinary(fileName);
  ContactPatch cp_from_bin;
  cp_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(cp3 == cp_from_bin);
}

//BOOST_AUTO_TEST_CASE(contact_phase)
//{
//  ContactPhase4 cp, cp_test;
//  for(ContactPhase4::ContactPatchMap::iterator it = cp.contact_patches().begin();
//      it !=  cp.contact_patches().end(); ++it)
//  {
//    it->second.contactModel().m_mu = 0.3;
//    it->second.contactModel().m_ZMP_radius = 0.01;
//    it->second.placement().setRandom();
//    it->second.contactModelPlacement().setRandom();
//    it->second.worldContactModelPlacement().setRandom();
//  }
//  ContactPhase4 cp2(cp);
//  BOOST_CHECK(cp == cp);
//  BOOST_CHECK(cp == cp2);
//  // test serialization
//  cp.saveAsText("serialization_cp_test.test");
//  cp_test.loadFromText("serialization_cp_test.test");
//  remove("serialization_cp_test.test");
//  BOOST_CHECK(cp == cp_test);
//}

//BOOST_AUTO_TEST_CASE(contact_sequence)
//{
//  ContactPhase4 cp;
//  ContactSequence4 cs(1);
//  ContactSequence4 cs_test(0);
//  cs.m_contact_phases[0] = cp;
//  // test serialization
//  cs.saveAsText("serialization_cs_test.test");
//  cs_test.loadFromText("serialization_cs_test.test");
//  remove("serialization_cp_test.test");
//  BOOST_CHECK(cs == cs_test);
//}

BOOST_AUTO_TEST_SUITE_END()
