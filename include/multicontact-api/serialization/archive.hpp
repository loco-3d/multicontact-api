// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_serialization_archive_hpp__
#define __multicontact_api_serialization_archive_hpp__

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/version.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

const unsigned int API_VERSION =
    2;  // must be increased everytime the save() method of a class is modified

// Macro used to define the serialization version of a templated class
#define MULTICONTACT_API_DEFINE_CLASS_TEMPLATE_VERSION(Template, Type) \
  namespace boost {                                                    \
  namespace serialization {                                            \
  template <Template>                                                  \
  struct version<Type> {                                               \
    static constexpr unsigned int value = API_VERSION;                 \
  };                                                                   \
  template <Template>                                                  \
  constexpr unsigned int version<Type>::value;                         \
  }                                                                    \
  }

namespace multicontact_api {
namespace serialization {

template <class Derived>
struct Serializable {
 private:
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

 public:
  /// \brief Loads a Derived object from a text file.
  void loadFromText(const std::string& filename) {
    std::ifstream ifs(filename.c_str());
    if (ifs) {
      boost::archive::text_iarchive ia(ifs);
      ia >> derived();
    } else {
      const std::string exception_message(filename +
                                          " does not seem to be a valid file.");
      throw std::invalid_argument(exception_message);
    }
  }

  /// \brief Saved a Derived object as a text file.
  void saveAsText(const std::string& filename) const {
    std::ofstream ofs(filename.c_str());
    if (ofs) {
      boost::archive::text_oarchive oa(ofs);
      oa << derived();
    } else {
      const std::string exception_message(filename +
                                          " does not seem to be a valid file.");
      throw std::invalid_argument(exception_message);
    }
  }

  /// \brief Loads a Derived object from an XML file.
  void loadFromXML(const std::string& filename, const std::string& tag_name) {
    assert(!tag_name.empty());
    std::ifstream ifs(filename.c_str());
    if (ifs) {
      boost::archive::xml_iarchive ia(ifs);
      ia >> boost::serialization::make_nvp(tag_name.c_str(), derived());
    } else {
      const std::string exception_message(filename +
                                          " does not seem to be a valid file.");
      throw std::invalid_argument(exception_message);
    }
  }

  /// \brief Saved a Derived object as an XML file.
  void saveAsXML(const std::string& filename,
                 const std::string& tag_name) const {
    assert(!tag_name.empty());
    std::ofstream ofs(filename.c_str());
    if (ofs) {
      boost::archive::xml_oarchive oa(ofs);
      oa << boost::serialization::make_nvp(tag_name.c_str(), derived());
    } else {
      const std::string exception_message(filename +
                                          " does not seem to be a valid file.");
      throw std::invalid_argument(exception_message);
    }
  }

  /// \brief Loads a Derived object from an binary file.
  void loadFromBinary(const std::string& filename) {
    std::ifstream ifs(filename.c_str());
    if (ifs) {
      boost::archive::binary_iarchive ia(ifs);
      ia >> derived();
    } else {
      const std::string exception_message(filename +
                                          " does not seem to be a valid file.");
      throw std::invalid_argument(exception_message);
    }
  }

  /// \brief Saved a Derived object as an binary file.
  void saveAsBinary(const std::string& filename) const {
    std::ofstream ofs(filename.c_str());
    if (ofs) {
      boost::archive::binary_oarchive oa(ofs);
      oa << derived();
    } else {
      const std::string exception_message(filename +
                                          " does not seem to be a valid file.");
      throw std::invalid_argument(exception_message);
    }
  }
};

}  // namespace serialization

}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_serialization_archive_hpp__
