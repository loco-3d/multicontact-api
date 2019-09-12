// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/geometry/linear-cone.hpp"
#include "multicontact-api/geometry/second-order-cone.hpp"

using namespace multicontact_api::geometry;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(WrenchConeTest) {
  using std::fabs;
  typedef ForceCone::Matrix3x Matrix3x;
  typedef WrenchCone::Matrix6x Matrix6x;
  typedef ForceCone::Index Index;
  typedef ForceCone::SE3 SE3;
  typedef ForceCone::Scalar Scalar;

  const Index size = 10;
  const Matrix3x rays(Matrix3x::Random(3, size));

  ForceCone C3(rays);
  BOOST_CHECK(C3.size() == size);
  BOOST_CHECK(C3.isApprox(C3));

  // Transform a ForceCone to a Wrench Cone
  const SE3 M1(SE3::Identity());
  WrenchCone C6 = C3.SE3ActOn(M1);
  BOOST_CHECK(C6.linear().isApprox(C3.rays()));

  const SE3 M2(SE3::Random());
  C6 = C3.SE3ActOn(M2);
  WrenchCone C6_inv = C6.SE3ActOn(M2.inverse());

  BOOST_CHECK(C6_inv.linear().isApprox(C3.rays()));

  // Export force cone to txt file
  const std::string txt_filename = "C3_in.txt";
  C3.saveAsText(txt_filename);

  ForceCone C3_from_txt(0);
  C3_from_txt.loadFromText(txt_filename);

  BOOST_CHECK(C3_from_txt.isApprox(C3));

  // Export gmm to xml file
  const std::string xml_filename = "C3_in.xml";
  C3.saveAsXML(xml_filename, "ForceCone");

  ForceCone C3_from_xml(0);
  C3_from_xml.loadFromXML(xml_filename, "ForceCone");

  BOOST_CHECK(C3_from_xml.isApprox(C3));

  // Export gmm to binary file
  const std::string bin_filename = "gmm_in.bin";
  C3.saveAsBinary(bin_filename);

  ForceCone C3_from_bin(0);
  C3_from_bin.loadFromBinary(bin_filename);

  BOOST_CHECK(C3_from_bin.isApprox(C3));
}

BOOST_AUTO_TEST_CASE(SecondOrderConeTest) {
  const double mu = 0.3;
  const SOC3d::VectorD direction(0., 0., 1.);

  SOC3d cone3 = SOC3d::RegularCone(mu, direction);

  BOOST_CHECK(cone3.check(direction));

  SOC3d::VectorD x(direction);
  x[0] = mu;
  BOOST_CHECK(cone3.check(x));

  x[0] = 0.;
  x[1] = mu;
  BOOST_CHECK(cone3.check(x));
}

BOOST_AUTO_TEST_SUITE_END()
