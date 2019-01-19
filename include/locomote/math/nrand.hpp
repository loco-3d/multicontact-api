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
#ifndef __locomote_math_nrand_hpp__
#define __locomote_math_nrand_hpp__

#include <cmath>
#include <cstdlib>
#include <boost/math/constants/constants.hpp>

namespace locomote
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

#endif // ifndef __locomote_math_nrand_hpp__
