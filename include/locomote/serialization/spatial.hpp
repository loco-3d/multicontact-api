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

#ifndef __locomote_serialization_spatial_hpp__
#define __locomote_serialization_spatial_hpp__

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/force.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost{
  
  namespace serialization{
    
    template <class Archive, typename _Scalar, int _Options>
    void save(Archive & ar, const pinocchio::SE3Tpl<_Scalar,_Options> & M, const unsigned int /*version*/)
    {
      ar & make_nvp("translation",make_array(M.translation().data(),3));
      ar & make_nvp("rotation",make_array(M.rotation().data(),9));
    }
    
    template <class Archive, typename _Scalar, int _Options>
    void load(Archive & ar, pinocchio::SE3Tpl<_Scalar,_Options> & M, const unsigned int /*version*/)
    {
      ar >> make_nvp("translation",make_array(M.translation().data(),3));
      ar >> make_nvp("rotation",make_array(M.rotation().data(),9));
    }
    
    template <class Archive, typename _Scalar, int _Options>
    void serialize(Archive & ar, pinocchio::SE3Tpl<_Scalar,_Options> & M, const unsigned int version)
    {
      split_free(ar,M,version);
    }
    
    template <class Archive, typename _Scalar, int _Options>
    void save(Archive & ar, const pinocchio::ForceTpl<_Scalar,_Options> & f, const unsigned int /*version*/)
    {
      ar & make_nvp("linear",make_array(f.linear().data(),3));
      ar & make_nvp("angular",make_array(f.angular().data(),3));
    }
    
    template <class Archive, typename _Scalar, int _Options>
    void load(Archive & ar, pinocchio::ForceTpl<_Scalar,_Options> & f, const unsigned int /*version*/)
    {
      ar >> make_nvp("linear",make_array(f.linear().data(),3));
      ar >> make_nvp("angular",make_array(f.angular().data(),3));
    }
    
    template <class Archive, typename _Scalar, int _Options>
    void serialize(Archive & ar, pinocchio::ForceTpl<_Scalar,_Options> & f, const unsigned int version)
    {
      split_free(ar,f,version);
    }
    
  }
  
}

#endif // ifndef __locomote_serialization_spatial_hpp__
