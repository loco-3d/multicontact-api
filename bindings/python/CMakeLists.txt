# Copyright (c) 2015-2018, CNRS
# Authors: Justin Carpentier <jcarpent@laas.fr>

SET(${PROJECT_NAME}_PYTHON_SOURCES
  module.cpp
  geometry/ellipsoid.cpp
  geometry/linear-cone.cpp
  geometry/second-order-cone.cpp
  scenario/contact-model.cpp
  scenario/contact-patch.cpp
  scenario/contact-phase.cpp
  #scenario/contact-sequence.cpp
  scenario/enums.cpp
  )

ADD_LIBRARY(${PY_NAME} SHARED ${${PROJECT_NAME}_PYTHON_SOURCES} ${${PROJECT_NAME}_PYTHON_HEADERS})
ADD_SOURCE_GROUP(${PROJECT_NAME}_PYTHON_SOURCES)
PKG_CONFIG_USE_DEPENDENCY(${PY_NAME} eigenpy)
PKG_CONFIG_USE_DEPENDENCY(${PY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${PY_NAME} curves)
TARGET_LINK_LIBRARIES(${PY_NAME} ${Boost_SERIALIZATION_LIBRARIES})
TARGET_LINK_BOOST_PYTHON(${PY_NAME})
SET_TARGET_PROPERTIES(${PY_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME})

INSTALL(TARGETS ${PY_NAME} DESTINATION  ${${PY_NAME}_INSTALL_DIR})

ADD_SUBDIRECTORY(${PY_NAME})
