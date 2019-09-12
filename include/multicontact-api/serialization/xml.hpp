// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_serialization_xml_hpp__
#define __multicontact_api_serialization_xml_hpp__

namespace multicontact_api {
namespace serialization {
template <class C>
struct serialize {
  template <class Archive>
  static operator(Archive& ar, C& c, const unsigned int version);
};

}  // namespace serialization

}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_serialization_xml_hpp__
