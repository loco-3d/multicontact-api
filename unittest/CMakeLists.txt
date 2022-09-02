# Copyright (c) 2015-2020, CNRS Authors: Justin Carpentier <jcarpent@laas.fr>,
# Guilhem Saurel

add_definitions(-DBOOST_TEST_DYN_LINK)

set(${PROJECT_NAME}_TESTS geometry scenario)

foreach(TEST ${${PROJECT_NAME}_TESTS})
  add_unit_test(${TEST} "${TEST}.cpp")
  target_link_libraries(${TEST} ${PROJECT_NAME} Boost::unit_test_framework)
endforeach(TEST ${${PROJECT_NAME}_TESTS})

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
