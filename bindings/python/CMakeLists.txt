# Copyright (c) 2015-2020, CNRS Authors: Justin Carpentier <jcarpent@laas.fr>,
# Guilhem Saurel

set(${PROJECT_NAME}_PYTHON_SOURCES
    module.cpp
    geometry/ellipsoid.cpp
    geometry/linear-cone.cpp
    geometry/second-order-cone.cpp
    scenario/contact-model.cpp
    scenario/contact-patch.cpp
    scenario/contact-phase.cpp
    scenario/contact-sequence.cpp
    scenario/enums.cpp)

add_library(${PY_NAME} SHARED ${${PROJECT_NAME}_PYTHON_SOURCES}
                              ${${PROJECT_NAME}_PYTHON_HEADERS})

add_source_group(${PROJECT_NAME}_PYTHON_SOURCES)

target_link_libraries(${PY_NAME} ${PROJECT_NAME} eigenpy::eigenpy)

set_target_properties(${PY_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
                                            SOVERSION ${PROJECT_VERSION})
target_compile_options(${PY_NAME} PRIVATE "-Wno-conversion")
target_compile_options(${PY_NAME} PRIVATE "-Wno-enum-compare")

install(TARGETS ${PY_NAME} DESTINATION ${${PY_NAME}_INSTALL_DIR})

set(${PROJECT_NAME}_PYTHON_FILES __init__.py)

foreach(file ${${PROJECT_NAME}_PYTHON_FILES})
  python_install_on_site(${PY_NAME} ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON_FILES})
