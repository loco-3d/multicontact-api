// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_geometry_fwd_hpp__
#define __multicontact_api_geometry_fwd_hpp__

namespace multicontact_api {
namespace geometry {

template <class T>
struct traits {};

template <typename _Scalar, int dim, int _Options = 0>
struct Ellipsoid;
typedef Ellipsoid<double, 3> Ellipsoid3d;

template <class Derived>
struct LinearConeBase;
template <typename _Scalar, int _dim, int _Options = 0>
struct LinearCone;
template <typename _Scalar, int _Options = 0>
struct ForceConeTpl;
typedef ForceConeTpl<double> ForceCone;
template <typename _Scalar, int _Options = 0>
struct WrenchConeTpl;
typedef WrenchConeTpl<double> WrenchCone;

template <typename _Scalar, int _dim, int _Options = 0>
struct SecondOrderCone;
typedef SecondOrderCone<double, 6> SOC6d;
typedef SecondOrderCone<double, 3> SOC3d;

}  // namespace geometry
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_geometry_fwd_hpp__
