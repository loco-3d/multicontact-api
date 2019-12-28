// Copyright (c) 2019-2020, CNRS
// Authors: Pierre Fernbach <pfernbac@laas.fr>

#ifndef __multicontact_api_geometry_curve_map_hpp__
#define __multicontact_api_geometry_curve_map_hpp__

#include <curves/curve_abc.h>
#include <map>
#include <string>

#include "multicontact-api/serialization/archive.hpp"

#include <boost/serialization/access.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/base_object.hpp>
#include <curves/serialization/registeration.hpp>


template <typename Curve>
struct CurveMap : public std::map<std::string,Curve> {

typedef CurveMap<Curve> CurveMap_t;
typedef std::map<std::string,Curve> Parent;

  // define operator == for map of shared ptr: start by checking if the ptr are same, otherwise check if the values are the sames
  bool operator==(const CurveMap_t& other) const {
    if(this->size() != other.size())
      return false;
    for(typename Parent::const_iterator it = this->begin() ; it != this->end() ; ++it){
      if(other.count(it->first) < 1)
        return false;
      if((it->second != other.at(it->first)) && !(it->second->isApprox(other.at(it->first).get())))
        return false;
    }
    return true;
  }

  bool operator!=(const CurveMap_t& other)const {
    return !(*this == other);
  }

 friend class boost::serialization::access;
 template <class Archive>
 void serialize(Archive& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Parent);
 }


};

#endif // CURVEMAP_HPP
