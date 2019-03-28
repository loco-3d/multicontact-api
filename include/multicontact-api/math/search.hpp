// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_math_search_hpp__
#define __multicontact_api_math_search_hpp__

#include <Eigen/Dense>

namespace multicontact_api
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

#endif // ifndef __multicontact_api_math_search_hpp__
