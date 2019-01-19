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

#ifndef __locomote_geometry_fwd_hpp__
#define __locomote_geometry_fwd_hpp__

namespace locomote
{
  namespace geometry
  {
    
    template<class T> struct traits {};
   
    template<typename _Scalar, int dim, int _Options=0> struct Ellipsoid;
    typedef Ellipsoid<double,3> Ellipsoid3d;
   
    template<class Derived> struct LinearConeBase;
    template<typename _Scalar, int _dim, int _Options=0> struct LinearCone;
    template<typename _Scalar, int _Options=0> struct ForceConeTpl;
    typedef ForceConeTpl<double> ForceCone;
    template<typename _Scalar, int _Options=0> struct WrenchConeTpl;
    typedef WrenchConeTpl<double> WrenchCone;
    
    template<typename _Scalar, int _dim, int _Options=0> struct SecondOrderCone;
    typedef SecondOrderCone<double,6> SOC6d;
    typedef SecondOrderCone<double,3> SOC3d;
    
    
  }
}

#endif // ifndef __locomote_geometry_fwd_hpp__
