# Copyright (c) 2015-2020, CNRS
# Authors: Justin Carpentier <jcarpent@laas.fr>, Guilhem Saurel

SET(${PROJECT_NAME}_PYTHON_SOURCES
  module.cpp
  geometry/ellipsoid.cpp
  geometry/linear-cone.cpp
  geometry/second-order-cone.cpp
  scenario/contact-model.cpp
  scenario/contact-patch.cpp
  scenario/contact-phase.cpp
  scenario/contact-sequence.cpp
  scenario/enums.cpp
  )

ADD_LIBRARY(${PY_NAME} SHARED ${${PROJECT_NAME}_PYTHON_SOURCES} ${${PROJECT_NAME}_PYTHON_HEADERS})

ADD_SOURCE_GROUP(${PROJECT_NAME}_PYTHON_SOURCES)

TARGET_LINK_LIBRARIES(${PY_NAME} ${PROJECT_NAME} eigenpy::eigenpy)
TARGET_LINK_BOOST_PYTHON(${PY_NAME})

SET_TARGET_PROPERTIES(${PY_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME} SOVERSION ${PROJECT_VERSION})
TARGET_COMPILE_OPTIONS(${PY_NAME} PRIVATE "-Wno-conversion")
TARGET_COMPILE_OPTIONS(${PY_NAME} PRIVATE "-Wno-enum-compare")

INSTALL(TARGETS ${PY_NAME} DESTINATION  ${${PY_NAME}_INSTALL_DIR})

SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  )

FOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME} ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})
