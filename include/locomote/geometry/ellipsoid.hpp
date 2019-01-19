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

#ifndef __locomote_geometry_ellipsoid_hpp__
#define __locomote_geometry_ellipsoid_hpp__

#include <Eigen/Dense>
#include <iostream>

#include "locomote/geometry/fwd.hpp"

namespace locomote {
  namespace geometry
  {
    template<typename _Scalar, int _dim, int _Options>
    struct Ellipsoid
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef _Scalar Scalar;
      enum { dim = _dim };
      enum { Options = _Options };
      
      typedef Eigen::Matrix<Scalar,dim,dim,Options> Matrix;
      typedef Eigen::Matrix<Scalar,dim,1,Options> Vector;
      
      Ellipsoid(const Matrix & A, const Vector & center)
      : m_A(A)
      , m_center(center)
      {}
      
      Scalar lhsValue(const Vector & point) const { return (m_A*(point-m_center)).norm(); }
      
      const Matrix & A() const { return m_A; }
      Matrix & A() { return m_A; }
      const Vector & center() const { return m_center; }
      Vector & center() { return m_center; }
      
      void disp(std::ostream & os) const
      {
        os
        << "A:\n" << m_A << std::endl
        << "center: " << m_center.transpose() << std::endl;
      }
      
      friend std::ostream & operator << (std::ostream & os,const Ellipsoid & E)
      {
        E.disp(os);
        return os;
      }
      
    protected:
      /// \brief
      Matrix m_A;
      
      /// \brief Center of the ellipsoid expressed in the global frame.
      Vector m_center;
    };
  }
}

#endif // ifndef __locomote_geometry_ellipsoid_hpp__
