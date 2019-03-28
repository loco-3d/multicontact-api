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
  scenario/contact-sequence.cpp
  scenario/enums.cpp
  trajectories/trajectories.cpp
  )

ADD_LIBRARY(${PY_NAME} SHARED ${${PROJECT_NAME}_PYTHON_SOURCES} ${${PROJECT_NAME}_PYTHON_HEADERS})
ADD_SOURCE_GROUP(${PROJECT_NAME}_PYTHON_SOURCES)
PKG_CONFIG_USE_DEPENDENCY(${PY_NAME} eigenpy)
PKG_CONFIG_USE_DEPENDENCY(${PY_NAME} pinocchio)
TARGET_LINK_LIBRARIES(${PY_NAME} ${Boost_SERIALIZATION_LIBRARIES})
TARGET_LINK_BOOST_PYTHON(${PY_NAME})

INSTALL(TARGETS ${PY_NAME} DESTINATION  ${${PY_NAME}_INSTALL_DIR})

SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  )

FOREACH(PYTHON_FILE ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_BUILD(. ${PYTHON_FILE})
  INSTALL(FILES ${${PY_NAME}_PYTHON_FILES} DESTINATION ${${PY_NAME}_INSTALL_DIR})
ENDFOREACH(PYTHON_FILE ${${PROJECT_NAME}_PYTHON_FILES})
