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
#ifndef __locomote_math_cumsum_hpp__
#define __locomote_math_cumsum_hpp__

#include <Eigen/Dense>

namespace locomote
{
  namespace math
  {
    
    namespace details
    {
      template<typename DenseDerived1, typename DenseDerived2, bool IsVector = DenseDerived1::ColsAtCompileTime == 1 || DenseDerived1::RowsAtCompileTime == 1>
      struct CumSumAlgo
      {
        
        static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                        const Eigen::DenseBase<DenseDerived2> & res,
                        const int direction);
        
        template<int Direction>
        static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                        const Eigen::DenseBase<DenseDerived2> & res);
        
        
      };
      
      template<typename DenseDerived1, typename DenseDerived2>
      struct CumSumAlgo<DenseDerived1,DenseDerived2,true>
      {
        static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                        const Eigen::DenseBase<DenseDerived2> & res)
        {
          Eigen::DenseBase<DenseDerived2> & res_ = const_cast<Eigen::DenseBase<DenseDerived2> &>(res);
          res_[0] = dense[0];
          for(int k = 1; k < dense.size(); ++k) res_[k] = res_[k-1] + dense[k];
        }
      };
      
      template<typename DenseDerived1, typename DenseDerived2>
      struct CumSumAlgo<DenseDerived1,DenseDerived2,false>
      {
        
        template<int Direction, typename Dummy = void>
        struct Func
        {
          static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                          const Eigen::DenseBase<DenseDerived2> & res)
          {
            Eigen::DenseBase<DenseDerived2> & res_ = const_cast<Eigen::DenseBase<DenseDerived2> &>(res);
            res_.col(0) = dense.col(0);
            for(int k = 1; k < dense.cols(); ++k) res_.col(k) = res_.col(k-1) + dense.col(k);
          }
          
        };
        
        template<typename Dummy>
        struct Func<Eigen::Horizontal,Dummy>
        {
          static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                          const Eigen::DenseBase<DenseDerived2> & res)
          {
            Eigen::DenseBase<DenseDerived2> & res_ = const_cast<Eigen::DenseBase<DenseDerived2> &>(res);
            res_.row(0) = dense.row(0);
            for(int k = 1; k < dense.rows(); ++k) res_.row(k) = res_.row(k-1) + dense.row(k);
          }
          
        };
        
        template<int Direction>
        static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                        const Eigen::DenseBase<DenseDerived2> & res)
        {
          Func<Direction>::run(dense,res);
        }
        
        
        static void run(const Eigen::DenseBase<DenseDerived1> & dense,
                        const Eigen::DenseBase<DenseDerived2> & res,
                        const int direction)
        {
          if(direction == Eigen::Vertical)
            run<Eigen::Vertical>(dense,res);
          else
            run<Eigen::Horizontal>(dense,res);
        }
        
      };
      
    } // namespace details
    
    template<typename DenseDerived1, typename DenseDerived2>
    void cumsum(const Eigen::DenseBase<DenseDerived1> & dense, const Eigen::DenseBase<DenseDerived2> & res)
    {
      details::CumSumAlgo<DenseDerived1,DenseDerived2>::run(dense,res);
    }
    
    template<typename DenseDerived>
    typename DenseDerived::PlainObject cumsum(const Eigen::DenseBase<DenseDerived> & dense)
    {
      typename DenseDerived::PlainObject res(dense.rows(), dense.cols());
      details::CumSumAlgo<DenseDerived,typename DenseDerived::PlainObject>::run(dense,res);
      return res;
    }
  }
}

#endif // ifndef __locomote_math_cumsum_hpp__
