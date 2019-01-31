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
#ifndef __locomote_geometry_linear_cone_hpp__
#define __locomote_geometry_linear_cone_hpp__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>

#include "locomote/geometry/fwd.hpp"
#include "locomote/serialization/archive.hpp"
#include "locomote/serialization/eigen-matrix.hpp"

#define EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_ROWS_SIZE(TYPE,ROWS) \
  EIGEN_STATIC_ASSERT(TYPE::RowsAtCompileTime==ROWS, \
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)

namespace locomote
{
  namespace geometry
  {
    
    template<typename _Scalar, int _dim, int _Options>
    struct LinearCone : public serialization::Serializable< LinearCone<_Scalar,_dim,_Options> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef _Scalar Scalar;
      enum
      {
        dim = _dim,
        Options = _Options
      };
      typedef Eigen::Matrix<Scalar,dim,-1,Options> MatrixDx;
      typedef Eigen::Matrix<Scalar,dim,dim,Options> MatrixD;
      typedef Eigen::Matrix<Scalar,dim,1,Options> VectorD;
      typedef Eigen::DenseIndex Index;
      
      /// \brief Default constructor
      LinearCone() : m_rays() {}
      
      /// \brief Constructor from a set of rays
      template<typename EigenDerived>
      explicit LinearCone(const Eigen::MatrixBase<EigenDerived> & rays)
//      : m_rays(_dim,rays.cols())
      : m_rays(rays)
      {
//        EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(EigenDerived,MatrixDx)
//        for(int k=0; k<rays.cols(); ++k)
//          m_rays.col(k) = rays.col(k).normalized();
      }
      
      /// \brief Contrustor from a given size.
      explicit LinearCone(const Index size)
      : m_rays(_dim,size)
      {}
      
      /// \brief Copy constructor
      template<typename S2, int O2>
      LinearCone(const LinearCone<S2,dim,O2> & other)
      : m_rays(other.m_rays)
      {}
      
      void addRay(const VectorD & ray)
      {
        m_rays.conservativeResize(Eigen::NoChange_t(), m_rays.cols()+1);
        m_rays.template rightCols<1> () = ray.normalized();
      }
      
      template<typename EigenDerived>
      void stack(const Eigen::MatrixBase<EigenDerived> & rays)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_ROWS_SIZE(EigenDerived,dim);
        m_rays.conservativeResize(Eigen::NoChange_t(), m_rays.cols()+rays.cols());
        m_rays.rightCols(rays.cols()) = rays;
      }
      
      template<typename S2, int O2>
      void stack(const LinearCone<S2,dim,O2> & other)
      { stack(other.rays()); }
      
      
      /// \returns the rays of the linear cone.
      const MatrixDx & rays() const { return m_rays; }
      MatrixDx & rays() { return m_rays; }
      
      /// \returns the number of rays, i.e. the number of cols of m_rays
      Index size() const { return m_rays.cols(); }
      
      template<typename S2, int O2>
      bool operator==(const LinearCone<S2,dim,O2> & other) const
      { return m_rays == other.m_rays; }
      
      template<typename S2, int O2>
      bool operator!=(const LinearCone<S2,dim,O2> & other) const
      { return !(*this == other); }
      
      template<typename S2, int O2>
      bool isApprox(const LinearCone<S2,dim,O2> & other,
                    const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
      { return m_rays.isApprox(other.m_rays,prec); }
      
      void disp(std::ostream & os) const
      { os << "Rays:\n" << m_rays << std::endl; }
      
      friend std::ostream & operator << (std::ostream & os, const LinearCone & C)
      { C.disp(os); return os; }
      
    protected:
      
      /// \brief Rays of the linear cone
      MatrixDx m_rays;
      
    private:
      
      // Serialization of the class
      friend class boost::serialization::access;
      
      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        ar & boost::serialization::make_nvp("rays",m_rays);
      }
      
      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        ar >> boost::serialization::make_nvp("rays",m_rays);
      }
      
      BOOST_SERIALIZATION_SPLIT_MEMBER()
      
    };
    
    
    template<typename _Scalar, int _Options>
    struct ForceConeTpl : public LinearCone<_Scalar,3,_Options>
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef LinearCone<_Scalar,3,_Options> Base;
      typedef WrenchConeTpl<_Scalar,_Options> WrenchCone;
      typedef pinocchio::SE3Tpl<_Scalar,_Options> SE3;
      using typename Base::Scalar;
      enum { dim = Base::dim };
      using typename Base::MatrixDx;
      using typename Base::MatrixD;
      using typename Base::VectorD;
      using typename Base::Index;
      using Base::size;
      using Base::rays;
      using Base::operator==;
      using Base::operator!=;
//      using Base::isApprox; // Leads to a bug with clang
      
      typedef MatrixDx Matrix3x;
      typedef VectorD Vector3;
      typedef Eigen::AngleAxis<Scalar> AngleAxis;
      
      /// \brief Default constructor
      ForceConeTpl() : Base() {}
      
      template<typename EigenDerived>
      explicit ForceConeTpl(const Eigen::MatrixBase<EigenDerived> & rays)
      : Base(rays) {}
      
      explicit ForceConeTpl(const Index size) : Base(size) {}
      
      /// \returns a linear cone built from a friction coefficient and the number of rays along the Z axis.
      static ForceConeTpl RegularCone(const Scalar mu, const VectorD & direction, const int num_rays, const Scalar theta_offset = 0.)
      {
        assert(mu >= 0. && "mu must be positive");
        assert(num_rays >= 1 && "The number of rays must be at least one");
        
        const VectorD normalized_direction(direction.normalized());
        ForceConeTpl cone(num_rays);
        
        const Scalar angle = (2.*M_PI) / num_rays;
        
        const MatrixD Po(MatrixD::Identity() - normalized_direction * normalized_direction.transpose());
        
        const MatrixD rot_offset(AngleAxis(theta_offset,normalized_direction).toRotationMatrix());
        const VectorD init_direction(rot_offset*(Po * VectorD::Ones()).normalized());
        const MatrixD rot(AngleAxis(angle,normalized_direction).toRotationMatrix());
        
        
        VectorD ray((direction + mu * init_direction).normalized());
        
        for(int k = 0; k < num_rays; ++k)
        {
          cone.rays().col(k) = ray;
          if (k != num_rays-1) ray = rot * ray;
        }
        
        return cone;
      }
      
      WrenchCone SE3ActOn(const SE3 & M) const
      {
        WrenchCone res(size());
        typedef typename WrenchCone::MatrixDx::ColXpr Col6Xpr;
        typedef typename MatrixDx::ConstColXpr ConstCol3Xpr;
        
        const typename SE3::Matrix3 & R = M.rotation();
        const typename SE3::Vector3 & t = M.translation();
        
        for(Index k = 0; k < size(); ++k)
        {
          ConstCol3Xpr in_col = rays().col(k);
          Col6Xpr out_col = res.rays().col(k);
          
          out_col.template head<3>() = R * in_col;
          out_col.template tail<3>() = t.cross(out_col.template head<3>());
        }
        
        return res;
      }
      
      template<typename S2, int O2>
      bool isApprox(const ForceConeTpl<S2,O2> & other,
                    const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
      { return Base::isApprox(other,prec); }
      
      operator WrenchCone() const
      {
        WrenchCone res(size());
        res.rays().template topRows<3>() = rays();
        res.rays().template bottomRows<3>().setZero();
        return res;
      }
    };
    
    
    template<typename _Scalar, int _Options>
    struct WrenchConeTpl : public LinearCone<_Scalar,6,_Options>
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef LinearCone<_Scalar,6,_Options> Base;
      typedef ForceConeTpl<_Scalar,_Options> ForceCone;
      typedef pinocchio::SE3Tpl<_Scalar,_Options> SE3;
      using typename Base::Scalar;
      enum { dim = Base::dim };
      using typename Base::MatrixDx;
      using typename Base::VectorD;
      using typename Base::Index;
      using Base::size;
      using Base::rays;
      using Base::operator==;
      using Base::operator!=;
      //      using Base::isApprox; // Leads to a bug with clang
      
      typedef MatrixDx Matrix6x;
      typedef VectorD Vector6;
      
      typedef typename ForceCone::Matrix3x Matrix3x;
      
      typedef typename Matrix6x::template NRowsBlockXpr<3>::Type LinearBlock;
      typedef typename Matrix6x::template ConstNRowsBlockXpr<3>::Type ConstLinearBlock;
      
      typedef LinearBlock AngularBlock;
      typedef ConstLinearBlock ConstAngularBlock;
      
      /// \brief Default constructor
      WrenchConeTpl() : Base() {}
      
      /// \brief Constructor from a set of rays.
      template<typename EigenDerived>
      explicit WrenchConeTpl(const Eigen::MatrixBase<EigenDerived> & rays)
      : Base(rays)
      {}
  
      /// \brief Constructs a WrenchCone of a given size.
      explicit WrenchConeTpl(const Index size) : Base(size) {}
      
      /// \brief Constructs a WrenchCone of a given size.
      template<typename S2, int O2>
      explicit WrenchConeTpl(const ForceConeTpl<S2,O2> & force_cone)
      : Base(force_cone.size())
      {
        rays().template topRows<3>() = force_cone.rays();
        rays().template bottomRows<3>().setZero();
      }
      
      /// \brief Copy constructor
      template<typename S2, int O2>
      WrenchConeTpl(const WrenchConeTpl<S2,O2> & other) : Base(other)
      {}
      
      WrenchConeTpl SE3ActOn(const SE3 & M) const
      {
        WrenchConeTpl res(size());
        typedef typename MatrixDx::ColXpr Col6Xpr;
        typedef typename MatrixDx::ConstColXpr ConstCol6Xpr;
        
        const typename SE3::Matrix3 & R = M.rotation();
        const typename SE3::Vector3 & t = M.translation();
        
        for(Index k = 0; k < size(); ++k)
        {
          ConstCol6Xpr in_col = rays().col(k);
          Col6Xpr out_col = res.rays().col(k);
          
          out_col.template head<3>() = R * in_col.template head<3>();
          out_col.template tail<3>() = t.cross(out_col.template head<3>()) + R * in_col.template tail<3>();
        }
        
        return res;
      }
      
      template<typename S2, int O2>
      bool isApprox(const WrenchConeTpl<S2,O2> & other,
                    const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
      { return Base::isApprox(other,prec); }
      
      ConstLinearBlock linear() const { return rays().template topRows<3> (); }
      LinearBlock linear() { return rays().template topRows<3> (); }
      
      ConstAngularBlock angular() const { return rays().template bottomRows<3> (); }
      AngularBlock angular() { return rays().template bottomRows<3> (); }
      
      ForceCone toForceCone() const { return ForceCone(linear()); }
      
    };
  }
}

#endif // ifndef __locomote_geometry_linear_cone_hpp__
