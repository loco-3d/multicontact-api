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

#ifndef __locomote_serialization_aligned_vector_hpp__
#define __locomote_serialization_aligned_vector_hpp__

#include <pinocchio/container/aligned-vector.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost{
  
  namespace serialization{
    
    template <class Archive, typename T>
    void save(Archive & ar, const se3::container::aligned_vector<T> & v, const unsigned int version)
    {
      typedef typename se3::container::aligned_vector<T>::vector_base vector_base;
      save(ar, *static_cast<const vector_base*>(&v), version);
    }
    
    template <class Archive, typename T>
    void load(Archive & ar, se3::container::aligned_vector<T> & v, const unsigned int version)
    {
      typedef typename se3::container::aligned_vector<T>::vector_base vector_base;
      load(ar, *static_cast<vector_base*>(&v), version);
    }
    
    template <class Archive, typename T>
    void serialize(Archive & ar, se3::container::aligned_vector<T> & v, const unsigned int version)
    {
      split_free(ar,v,version);
    }
    
  }
  
}

#endif // ifndef __locomote_serialization_aligned_vector_hpp__
