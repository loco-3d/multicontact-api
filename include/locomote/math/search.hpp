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
#ifndef __locomote_math_search_hpp__
#define __locomote_math_search_hpp__

#include <Eigen/Dense>

namespace locomote
{
  namespace math
  {

    template<class DenseDerived>
    Eigen::DenseIndex binarySearch(const Eigen::DenseBase<DenseDerived> & vector,
                                   const typename DenseDerived::Scalar & value)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(DenseDerived);
      
      int left_id = 0;
      int right_id = vector.ssize()-1;
      
      while(left_id <= right_id)
      {
        int middle_id = left_id + (right_id - left_id)/2;
        if(vector[middle_id] < value)
          left_id = middle_id+1;
        else if(vector[middle_id] > value)
          right_id = middle_id-1;
        else
          return middle_id;
      }
      
      return left_id-1;
    }
  }
}

#endif // ifndef __locomote_math_search_hpp__
