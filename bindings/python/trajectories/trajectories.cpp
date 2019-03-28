// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/trajectories/cubic-hermite-spline.hpp"

namespace multicontact_api
{
  namespace python
  {
    void exposeTrajectories()
    {
      using namespace trajectories;
      typedef CubicHermiteSplineTpl<double,3> CubicHermiteSpline3;
      typedef CubicHermiteSplineTpl<double,Eigen::Dynamic> CubicHermiteSpline;

      CubicHermiteSplinePythonVisitor<CubicHermiteSpline3>::expose("CubicHermiteSpline3");
      CubicHermiteSplinePythonVisitor<CubicHermiteSpline>::expose("CubicHermiteSpline");
      exposeSplineAlgos();
    }
  }
}
