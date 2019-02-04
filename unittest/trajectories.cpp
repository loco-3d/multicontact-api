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

#define BOOST_TEST_MODULE TrajectoriesTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "locomote/trajectories/cubic-hermite-spline.hpp"

using namespace locomote::trajectories;


BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(HermiteSpline)
{
  typedef CubicHermiteSplineTpl<double,3> CubicHermiteSpline3;
  typedef CubicHermiteSpline3::VectorX VectorX;
  typedef CubicHermiteSpline3::MatrixDx MatrixDx;
  typedef CubicHermiteSpline3::Index Index;
  
  // Test default constructor
  {
    CubicHermiteSpline3 spline;
    BOOST_CHECK(spline.size() == 0);
    CubicHermiteSpline3 spline2(0);
    BOOST_CHECK(spline.size() == 0);
  }
  
  const int size = 20;
  
  const MatrixDx points(MatrixDx::Random(3,size));
  const MatrixDx derivatives(MatrixDx::Random(3,size));
  const VectorX absicca(VectorX::LinSpaced(size,0.,1.));
  
  CubicHermiteSpline3 spline(absicca, points, derivatives);
  
  VectorX p0, m0;
  spline.eval(0.,p0,m0);
  VectorX p1, m1;
  spline.eval(1.,p1,m1);
  
  BOOST_CHECK(p0.isApprox(points.col(0)));
  BOOST_CHECK(m0.isApprox(derivatives.col(0)));
  BOOST_CHECK(p1.isApprox(points.rightCols<1>()));
  BOOST_CHECK(m1.isApprox(derivatives.rightCols<1>()));
  
  for(Index k = 0; k < size; ++k)
  {
    VectorX p,m;
    spline.eval(absicca[k],p,m);
    BOOST_CHECK(p.isApprox(points.col(k)));
    BOOST_CHECK(m.isApprox(derivatives.col(k)));
  }
  
  spline.derivatives().setZero();
  for(Index k = 0; k < spline.numIntervals(); ++k)
  {
    double t0 = absicca[k];
    double t1 = absicca[k+1];
    
    VectorX p0 = points.col(k);
    VectorX p1 = points.col(k+1);
    VectorX p_ref(0.5*(p0+p1));
    
    double t = 0.5*(t0+t1);
    VectorX p,m;
    spline.eval(t,p,m);
    BOOST_CHECK(p.isApprox(p_ref));
    BOOST_CHECK(m.isZero());
  }
  
  // Test copy constructor
  CubicHermiteSpline3 spline2(spline);
  BOOST_CHECK(spline2 == spline);
  
  // Test spline with only a single trajectorie defined by two end points
  {
    CubicHermiteSpline3 spline(2);
    spline.points().setZero();
    spline.derivatives().setZero();
    
    CubicHermiteSpline3::VectorX absicca = spline.absicca();
    absicca[0] = 0.; absicca[1] = 1.;
    spline.setAbsicca(absicca);
    
    CubicHermiteSpline3::VectorD value, dvalue;
    spline.eval(0.,value,dvalue);
    BOOST_CHECK(value.isApprox(CubicHermiteSpline3::VectorD::Zero()));
    BOOST_CHECK(dvalue.isApprox(CubicHermiteSpline3::VectorD::Zero()));
  }

  // Test the addition and subtraction operators
  {
    const int size = 20;
  
    const MatrixDx points1(MatrixDx::Random(3,size));
    const MatrixDx derivatives1(MatrixDx::Random(3,size));
    const MatrixDx points2(MatrixDx::Random(3,size));
    const MatrixDx derivatives2(MatrixDx::Random(3,size));

    const VectorX absicca(VectorX::LinSpaced(size,0.,1.));
  
    CubicHermiteSpline3 spline1(absicca, points1, derivatives1);
    CubicHermiteSpline3 spline2(absicca, points2, derivatives2);

    const CubicHermiteSpline3 spline_res_a = spline1+spline2;
    const CubicHermiteSpline3 spline_res_s = spline1-spline2;

    
    VectorX p1,m1, p2, m2, p_res_a, m_res_a, p_res_s, m_res_s;
    VectorX evalPoints(VectorX::Random(size));  //Random Evaluation points in range[-1., 1.]
    evalPoints.array() += 1.0;     evalPoints /=2.0;   // Move evaluation points to [0.,1.]
    for(int i=0;i<size;i++)
    {
      double t = evalPoints[i];
      spline1.eval(t,p1,m1);
      spline2.eval(t,p2,m2);
      spline_res_a.eval(t,p_res_a,m_res_a);
      spline_res_s.eval(t,p_res_s,m_res_s);

      BOOST_CHECK(p_res_a.isApprox(p1+p2));
      BOOST_CHECK(m_res_a.isApprox(m1+m2));
      BOOST_CHECK(p_res_s.isApprox(p1-p2));
      BOOST_CHECK(m_res_s.isApprox(m1-m2));
    }
  }
    // Test spline recomputation at a different abscissa
  {
    const int size = 20;
    const int size2 = 33;
    const int neval = 100;
    const MatrixDx points(MatrixDx::Random(3,size));
    const MatrixDx derivatives(MatrixDx::Random(3,size));

    const VectorX absicca(VectorX::LinSpaced(size,0.,1.));
    VectorX absicca2(VectorX::LinSpaced(size2,0.,1.));
  
    CubicHermiteSpline3 spline1(absicca, points, derivatives);
    
    CubicHermiteSpline3 spline2 = createHermiteSplineAtAbsicca(spline1, absicca2);
    VectorX p1,m1, p2, m2;
    VectorX evalPoints(VectorX::Random(neval));  //Random Evaluation points in range[-1., 1.]
    evalPoints.array() += 1.0;     evalPoints /=2.0;   // Move evaluation points to [0.,1.]
    for(int i=0;i<size2;i++)
    {
      double t = absicca2[i];
      spline1.eval(t,p1,m1);
      spline2.eval(t,p2,m2);
      BOOST_CHECK(p1.isApprox(p2));
      BOOST_CHECK(m1.isApprox(m2));
    }
  } 
}

BOOST_AUTO_TEST_SUITE_END()
