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

# --- MACROS ------------------------------------------------------------------
MACRO(ADD_UNIT_TEST name PKGS)
  IF (BUILD_UNIT_TESTS)
    ADD_EXECUTABLE(${name} ${name})
  ELSE (BUILD_UNIT_TESTS)
    ADD_EXECUTABLE(${name} EXCLUDE_FROM_ALL ${name})
  ENDIF (BUILD_UNIT_TESTS)
  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${name} ${PKG})
  ENDFOREACH(PKG)

  TARGET_LINK_LIBRARIES(${name} ${Boost_LIBRARIES})

  ADD_TEST(NAME ${name} COMMAND ${name})
  ADD_DEPENDENCIES(check ${name})
ENDMACRO(ADD_UNIT_TEST)

# --- RULES -------------------------------------------------------------------
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
ADD_CUSTOM_TARGET(check COMMAND ${CMAKE_CTEST_COMMAND})

ADD_UNIT_TEST(geometry "eigen3;pinocchio")
ADD_UNIT_TEST(scenario "eigen3;pinocchio")
ADD_UNIT_TEST(trajectories "eigen3")
