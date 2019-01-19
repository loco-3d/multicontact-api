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

#ifndef __locomote_serialization_archive_hpp__
#define __locomote_serialization_archive_hpp__

#include "locomote/serialization/fwd.hpp"

#include <fstream>
#include <string>
#include <stdexcept>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>


namespace locomote
{
  namespace serialization
  {
    
    template<class Derived>
    struct Serializable
    {
    private:
      Derived & derived() { return *static_cast<Derived*>(this); }
      const Derived & derived() const { return *static_cast<const Derived*>(this); }
      
    public:
      /// \brief Loads a Derived object from a text file.
      void loadFromText(const std::string & filename) throw (std::invalid_argument)
      {
        std::ifstream ifs(filename.c_str());
        if(ifs)
        {
          boost::archive::text_iarchive ia(ifs);
          ia >> derived();
        }
        else
        {
          const std::string exception_message(filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
      }
      
      /// \brief Saved a Derived object as a text file.
      void saveAsText(const std::string & filename) const throw (std::invalid_argument)
      {
        std::ofstream ofs(filename.c_str());
        if(ofs)
        {
          boost::archive::text_oarchive oa(ofs);
          oa << derived();
        }
        else
        {
          const std::string exception_message(filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
      }
      
      /// \brief Loads a Derived object from an XML file.
      void loadFromXML(const std::string & filename, const std::string & tag_name) throw (std::invalid_argument)
      {
        assert(!tag_name.empty());
        std::ifstream ifs(filename.c_str());
        if(ifs)
        {
          boost::archive::xml_iarchive ia(ifs);
          ia >> boost::serialization::make_nvp(tag_name.c_str(),derived());
        }
        else
        {
          const std::string exception_message(filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
      }
      
      /// \brief Saved a Derived object as an XML file.
      void saveAsXML(const std::string & filename, const std::string & tag_name) const throw (std::invalid_argument)
      {
        assert(!tag_name.empty());
        std::ofstream ofs(filename.c_str());
        if(ofs)
        {
          boost::archive::xml_oarchive oa(ofs);
          oa << boost::serialization::make_nvp(tag_name.c_str(),derived());
        }
        else
        {
          const std::string exception_message(filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
      }
      
      /// \brief Loads a Derived object from an binary file.
      void loadFromBinary(const std::string & filename) throw (std::invalid_argument)
      {
        std::ifstream ifs(filename.c_str());
        if(ifs)
        {
          boost::archive::binary_iarchive ia(ifs);
          ia >> derived();
        }
        else
        {
          const std::string exception_message(filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
      }
      
      /// \brief Saved a Derived object as an binary file.
      void saveAsBinary(const std::string & filename) const throw (std::invalid_argument)
      {
        std::ofstream ofs(filename.c_str());
        if(ofs)
        {
          boost::archive::binary_oarchive oa(ofs);
          oa << derived();
        }
        else
        {
          const std::string exception_message(filename + " does not seem to be a valid file.");
          throw std::invalid_argument(exception_message);
        }
      }
    };
    
  }
  
}

#endif // ifndef __locomote_serialization_archive_hpp__
