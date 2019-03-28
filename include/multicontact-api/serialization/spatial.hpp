// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_serialization_spatial_hpp__
#define __multicontact_api_serialization_spatial_hpp__

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

#endif // ifndef __multicontact_api_serialization_spatial_hpp__
