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
#ifndef __locomote_geometry_second_order_cone_hpp__
#define __locomote_geometry_second_order_cone_hpp__

#include <Eigen/Dense>
#include <iostream>

#include "locomote/geometry/fwd.hpp"
#include "locomote/serialization/archive.hpp"
#include "locomote/serialization/eigen-matrix.hpp"

namespace locomote
{
  namespace geometry
  {
    
    template<typename _Scalar, int _dim, int _Options>
    struct SecondOrderCone : public serialization::Serializable< SecondOrderCone<_Scalar,_dim,_Options> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      enum
      {
        dim = _dim,
        Options = _Options
      };
      typedef _Scalar Scalar;
      typedef Eigen::Matrix<Scalar,dim,dim,Options> MatrixD;
      typedef Eigen::Matrix<Scalar,dim,1,Options> VectorD;
      typedef Eigen::DenseIndex DenseIndex;
      
      SecondOrderCone()
      : m_Q(MatrixD::Identity())
      , m_QPo(_dim,_dim)
      , m_direction(VectorD::Zero())
      , m_Pd(_dim,_dim)
      , m_Po(_dim,_dim)
      {
        m_direction[_dim-1] = 1.;
        computeProjectors();
      }
      
      SecondOrderCone(const MatrixD & Q, const VectorD & direction)
      : m_Q(Q)
      , m_QPo(_dim,_dim)
      , m_direction(direction.normalized())
      , m_Pd(_dim,_dim)
      , m_Po(_dim,_dim)
      {
        assert(direction.norm() >= Eigen::NumTraits<Scalar>::dummy_precision());
        assert((Q - Q.transpose()).isMuchSmallerThan(Q));
        computeProjectors();
      }
      
      ///
      /// \brief Build a regular cone from a given friction coefficient and a direction.
      ///
      /// \param mu Friction coefficient.
      /// \param direction Direction of the cone.
      ///
      /// \returns A second order cone.
      ///
      static SecondOrderCone RegularCone(const Scalar mu, const VectorD & direction)
      {
        assert(mu > 0 && "The friction coefficient must be non-negative");
        MatrixD Q(MatrixD::Zero());
        Q.diagonal().fill(1./mu);
        
        return SecondOrderCone(Q,direction);
      }
      
      template<typename S2, int O2>
      bool operator==(const SecondOrderCone<S2,dim,O2> & other) const
      {
        return m_Q == other.m_Q
        && m_direction == other.m_direction;
      }
      
      template<typename S2, int O2>
      bool operator!=(const SecondOrderCone<S2,dim,O2> & other) const
      { return !(*this == other); }
      
      /// \returns the value of lhs of the conic inequality
      Scalar lhsValue(const VectorD & point) const
      {
//        const VectorD x_Po(m_Po * point);
        return (m_QPo*point).norm();
      }
      
      /// \returns the value of rhs of the conic inequality
      Scalar rhsValue(const VectorD & point) const
      {
        return m_direction.dot(point);
      }
      
      /// \returns true if the point is inside the cone
      bool check(const VectorD & point, const Scalar factor = 1.) const
      { return lhsValue(point) <= factor * rhsValue(point); }
      
      /// \returns the direction of the cone.
      const VectorD & direction() const { return m_direction; }
      void setDirection(const VectorD & direction)
      {
        assert(direction.norm() >= Eigen::NumTraits<Scalar>::dummy_precision());
        m_direction = direction.normalized();
        computeProjectors();
      }
      
      template<typename S2, int O2>
      bool isApprox(const SecondOrderCone<S2,dim,O2> & other,
                    const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
      {
        return m_direction.isApprox(other.m_direction,prec)
        && m_Q.isApprox(other.m_Q,prec);
      }
      
      /// \returns the quadratic term of the lhs norm.
      const MatrixD & Q() const { return m_Q; }
      void setQ(const MatrixD & Q)
      {
        assert((Q - Q.transpose()).isMuchSmallerThan(Q));
        m_Q = Q;
        computeProjectors();
      }
      
      void disp(std::ostream & os) const
      {
        os
        << "Q:\n" << m_Q << std::endl
        << "direction: " << m_direction.transpose() << std::endl;
      }
      
      friend std::ostream & operator << (std::ostream & os, const SecondOrderCone & C)
      { C.disp(os); return os; }
      
    protected:
      
      inline void computeProjectors()
      {
        m_Pd = m_direction * m_direction.transpose();
        m_Po = MatrixD::Identity() - m_Pd;
        m_QPo.noalias() = m_Q * m_Po;
      }
      
      /// \brief Cholesky decomposition matrix reprensenting the conic norm
      MatrixD m_Q;
      /// \brief Cholesky decomposition projected on the orthogonal of m_direction
      MatrixD m_QPo;
      
      /// \brief Direction of the cone
      VectorD m_direction;
      
      /// \brief Projector along the direction of d
      MatrixD m_Pd;
      /// \brief Projector orthogonal to d
      MatrixD m_Po;
      
    private:
      
      // Serialization of the class
      friend class boost::serialization::access;
      
      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        ar & boost::serialization::make_nvp("quadratic_term",m_Q);
        ar & boost::serialization::make_nvp("direction",m_direction);
      }
      
      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        ar >> boost::serialization::make_nvp("quadratic_term",m_Q);
        ar >> boost::serialization::make_nvp("direction",m_direction);
        
        computeProjectors();
      }
      
      BOOST_SERIALIZATION_SPLIT_MEMBER()
      
    };
    
  }
}

#endif // ifndef __locomote_geometry_second_order_cone_hpp__
