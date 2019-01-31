# Copyright (c) 2015-2018, CNRS
# Authors: Justin Carpentier <jcarpent@laas.fr>
# Simplified BSD license :
#Redistribution and use in source and binary forms, with or without modification,
#are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice,
#this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#this list of conditions and the following disclaimer in the documentation
#and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)

SET(PROJECT_NAME locomote)
SET(PROJECT_DESCRIPTION "Multi-contact locomotion for multi-body systems")
SET(PROJECT_URL "TODO")
OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" FALSE)


SET(DOXYGEN_USE_MATHJAX YES)

IF(APPLE)
  SET(CMAKE_MACOSX_RPATH TRUE)
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
     set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  endif("${isSystemDir}" STREQUAL "-1")
ENDIF(APPLE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

SETUP_PROJECT()

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)

# --- OPTIONS ----------------------------------------
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
OPTION (BUILD_UNIT_TESTS "Build the unitary tests" ON)

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.0.0")

SET(BOOST_COMPONENTS unit_test_framework serialization)

IF(BUILD_PYTHON_INTERFACE)
  ADD_COMPILE_DEPENDENCY("eigenpy >= 1.3.1")
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  FINDPYTHON(2.7 EXACT REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS}) 
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()
# Path to boost headers
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MATH_HEADERS
    include/locomote/math/cumsum.hpp
    include/locomote/math/factorial.hpp
    include/locomote/math/nrand.hpp
    include/locomote/math/search.hpp
  )

SET(${PROJECT_NAME}_GEOMETRY_HEADERS
    include/locomote/geometry/fwd.hpp
    include/locomote/geometry/linear-cone.hpp
    include/locomote/geometry/second-order-cone.hpp
    include/locomote/geometry/ellipsoid.hpp
  )

SET(${PROJECT_NAME}_SERIALIZATION_HEADERS
    include/locomote/serialization/eigen-matrix.hpp
    include/locomote/serialization/aligned-vector.hpp
    include/locomote/serialization/spatial.hpp
    include/locomote/serialization/fwd.hpp
    include/locomote/serialization/xml.hpp
    include/locomote/serialization/archive.hpp
  )

SET(${PROJECT_NAME}_SCENARIO_HEADERS
    include/locomote/scenario/fwd.hpp
    include/locomote/scenario/contact-sequence.hpp
    include/locomote/scenario/contact-phase.hpp
    include/locomote/scenario/contact-patch.hpp
    include/locomote/scenario/contact-phase-humanoid.hpp
    include/locomote/scenario/constraint.hpp
    include/locomote/scenario/constraint-gmm.hpp
    include/locomote/scenario/constraint-soc.hpp
    include/locomote/scenario/contact-constraint.hpp
    include/locomote/scenario/contact-constraint-planar.hpp
    include/locomote/scenario/contact-model-planar.hpp
    include/locomote/scenario/ms-interval.hpp
  )


SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
    include/locomote/trajectories/fwd.hpp
    include/locomote/trajectories/cubic-hermite-spline.hpp
  )

SET(${PROJECT_NAME}_CONTAINER_HEADERS
    include/locomote/container/ref.hpp
  )

SET(HEADERS
    ${${PROJECT_NAME}_MATH_HEADERS}
    ${${PROJECT_NAME}_GEOMETRY_HEADERS}
    ${${PROJECT_NAME}_SERIALIZATION_HEADERS}
    ${${PROJECT_NAME}_SCENARIO_HEADERS}
    ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
    ${${PROJECT_NAME}_CONTAINER_HEADERS}
 )

LIST(REMOVE_DUPLICATES HEADERS)

#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/math")
#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/geometry")
#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/stats")
#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/serialization")
#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/scenario")
#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/muscod")
#MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/model")

SET(HEADERS_FULL_PATH "")
FOREACH(header ${HEADERS})
  LIST(APPEND HEADERS_FULL_PATH "${CMAKE_SOURCE_DIR}/${header}") 
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  GET_FILENAME_COMPONENT(headerPath ${header} PATH)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/${header}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${headerPath}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header ${HEADERS})



# --- BINDINGS ----------------------------------------------------------------
ADD_SUBDIRECTORY(bindings)

# --- UNIT TESTS ---------------------------------------------------------------
ADD_SUBDIRECTORY(unittest)

# --- PACKAGING ----------------------------------------------------------------
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
PKG_CONFIG_APPEND_BOOST_LIBS(serialization)

SETUP_PROJECT_FINALIZE()
