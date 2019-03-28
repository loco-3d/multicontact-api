// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_math_factorial_hpp__
#define __multicontact_api_math_factorial_hpp__

namespace multicontact_api
{
  namespace math
  {
    template<int N> factorial() { assert(N>0); return N * factorial<N-1>(); }
    template<0> factorial() { return 1; }
    template<1> factorial() { return 1; }
    template<2> factorial() { return 2; }
  }
}

#endif // ifndef __multicontact_api_math_factorial_hpp__
