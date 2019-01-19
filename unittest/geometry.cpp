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

#include "locomote/geometry/linear-cone.hpp"
#include "locomote/geometry/second-order-cone.hpp"

using namespace locomote::geometry;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(WrenchConeTest)
{
  using std::fabs;
  typedef ForceCone::Matrix3x Matrix3x;
  typedef WrenchCone::Matrix6x Matrix6x;
  typedef ForceCone::Index Index;
  typedef ForceCone::SE3 SE3;
  typedef ForceCone::Scalar Scalar;
  
  const Index size = 10;
  const Matrix3x rays(Matrix3x::Random(3,size));
  
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
  C3.saveAsXML(xml_filename,"ForceCone");

  ForceCone C3_from_xml(0);
  C3_from_xml.loadFromXML(xml_filename,"ForceCone");

  BOOST_CHECK(C3_from_xml.isApprox(C3));

  // Export gmm to binary file
  const std::string bin_filename = "gmm_in.bin";
  C3.saveAsBinary(bin_filename);

  ForceCone C3_from_bin(0);
  C3_from_bin.loadFromBinary(bin_filename);

  BOOST_CHECK(C3_from_bin.isApprox(C3));
}

BOOST_AUTO_TEST_CASE(SecondOrderConeTest)
{
  const double mu = 0.3;
  const SOC3d::VectorD direction(0.,0.,1.);
  
  SOC3d cone3 = SOC3d::RegularCone(mu,direction);
 
  BOOST_CHECK(cone3.check(direction));
  
  SOC3d::VectorD x(direction);
  x[0] = mu;
  BOOST_CHECK(cone3.check(x));
  
  x[0] = 0.; x[1] = mu;
  BOOST_CHECK(cone3.check(x));
  
}

BOOST_AUTO_TEST_SUITE_END()
