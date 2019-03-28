// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_math_nrand_hpp__
#define __multicontact_api_math_nrand_hpp__

#include <cmath>
#include <cstdlib>
#include <boost/math/constants/constants.hpp>

namespace multicontact_api
{
  namespace math
  {

    template<typename Scalar>
    inline Scalar nrand()
    {
      const Scalar two_pi = 2 * boost::math::constants::pi<Scalar>();
      const Scalar eps = std::numeric_limits<Scalar>::min();
      using std::rand;
      using std::cos;
      using std::sqrt;
      using std::log;

      Scalar u1, u2;
      const Scalar RAND_MAX_INV = (1.0 / RAND_MAX);
      do
      {
        u1 = (Scalar)rand() * RAND_MAX_INV;
        u2 = (Scalar)rand() * RAND_MAX_INV;
      }
      while(u1 <= eps);

      Scalar z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);

      return z0;

    }
  }
}

#endif // ifndef __multicontact_api_math_nrand_hpp__
