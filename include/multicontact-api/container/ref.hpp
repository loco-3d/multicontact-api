// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_container_ref_hpp__
#define __multicontact_api_container_ref_hpp__

#include <boost/ref.hpp>

namespace multicontact_api {
namespace container {
template <typename T>
class comparable_reference_wrapper : public boost::reference_wrapper<T> {
  typedef typename boost::reference_wrapper<T> base;

 public:
  typedef typename base::type type;

  // constructor
  explicit comparable_reference_wrapper(T& t) : boost::reference_wrapper<T>(t) {}

  // comparison operator
  bool operator==(const comparable_reference_wrapper& other) const { return this->get() == other.get(); }
};
}  // namespace container
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_container_ref_hpp__
