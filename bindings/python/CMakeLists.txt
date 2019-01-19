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

IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON_HEADERS
      fwd.hpp
      geometry/ellipsoid.hpp
      geometry/linear-cone.hpp
      geometry/second-order-cone.hpp
      geometry/expose-geometry.hpp
      serialization/archive.hpp
      scenario/expose-scenario.hpp
      scenario/contact-patch.hpp
      scenario/contact-model-planar.hpp
      scenario/contact-phase.hpp
      scenario/contact-phase-humanoid.hpp
      scenario/contact-sequence.hpp
      scenario/enums.hpp
      scenario/ms-interval.hpp
      container/array.hpp
      container/visitor.hpp
      container/reference-wrapper.hpp
      utils/base.hpp
      utils/printable.hpp
      trajectories/expose-trajectories.hpp
      trajectories/cubic-hermite-spline.hpp
    )

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

  LIST(APPEND HEADERS ${${PROJECT_NAME}_PYTHON_HEADERS})
  LIST(REMOVE_DUPLICATES HEADERS)

  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/geometry")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/scenario")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/serialization")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/container")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/utils")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/trajectories")

  FOREACH(header ${${PROJECT_NAME}_PYTHON_HEADERS})
    GET_FILENAME_COMPONENT(headerName ${header} NAME)
    GET_FILENAME_COMPONENT(headerPath ${header} PATH)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${header}
      ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/${header})
    INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${header}
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/bindings/python
            PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
  ENDFOREACH(header)

  # --- PyWrap library --- #
  SET(PYWRAP ${PROJECT_NAME}_pywrap)
  ADD_LIBRARY(${PYWRAP} SHARED ${${PROJECT_NAME}_PYTHON_SOURCES} ${${PROJECT_NAME}_PYTHON_HEADERS})
  ADD_HEADER_GROUP(${PROJECT_NAME}_PYTHON_HEADERS)
  ADD_SOURCE_GROUP(${PROJECT_NAME}_PYTHON_SOURCES)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} pinocchio)
  TARGET_LINK_LIBRARIES(${PYWRAP} ${Boost_SERIALIZATION_LIBRARIES})
  TARGET_LINK_BOOST_PYTHON(${PYWRAP})

  IF(APPLE)
    # We need to change the extension for python bindings
    SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES SUFFIX ".so")
  ENDIF(APPLE)

  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}")

  INSTALL(FILES
    "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}/lib${PYWRAP}.so"
    DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})

   # --- INSTALL SCRIPTS 
  SET(PYTHON_FILES
    __init__.py
    )

  FOREACH(python ${PYTHON_FILES})
    GET_FILENAME_COMPONENT(pythonFile ${python} NAME)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${python}
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})

    # Tag pyc file as generated.
    SET_SOURCE_FILES_PROPERTIES(
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      PROPERTIES GENERATED TRUE)

    EXECUTE_PROCESS(COMMAND
      ${PYTHON_EXECUTABLE} -m py_compile
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})

    # Clean generated files.
    SET_PROPERTY(
      DIRECTORY APPEND PROPERTY
      ADDITIONAL_MAKE_CLEAN_FILES
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c")

    INSTALL(FILES
      "${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${python}"
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
  ENDFOREACH(python)

ENDIF(BUILD_PYTHON_INTERFACE)

