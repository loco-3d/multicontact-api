# Copyright (c) 2015-2020, CNRS Authors: Justin Carpentier <jcarpent@laas.fr>,
# Guilhem Saurel

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG loco-3d)
set(PROJECT_NAME multicontact-api)
set(PROJECT_DESCRIPTION
    "API to define and store Contact phases and Contact Sequences")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/ide.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 ENFORCE)

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 2.7.12 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency(pinocchio REQUIRED)
add_project_dependency(ndcurves 1.0.0 REQUIRED)
if(NOT CURVES_WITH_PINOCCHIO_SUPPORT)
  message(
    FATAL_ERROR
      "you need to use a ndcurves version compiled with pinocchio support")
endif(NOT CURVES_WITH_PINOCCHIO_SUPPORT)
add_project_dependency(Boost REQUIRED COMPONENTS unit_test_framework
                       serialization)

# Main Library

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
                           INTERFACE $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio
                                                ndcurves::ndcurves)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  add_subdirectory(include/${CUSTOM_HEADER_DIR})
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
add_subdirectory(bindings)
add_subdirectory(unittest)
