#
# Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
#
# This file is part of the srsLTE library.
#
# srsLTE is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsLTE is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#


# - Try to find emanelte
# Once done this will define
#  emanelte_FOUND        - System has emanelte
#  emanelte_INCLUDE_DIRS - The emanelte include directories
#  emanelte_LIBRARIES    - The emanelte library

find_package(PkgConfig)
pkg_check_modules(PC_emanelte QUIET libemanelte)
IF(NOT emanelte_FOUND)

FIND_PATH(
    emanelte_INCLUDE_DIRS
    NAMES libemanelte
    HINTS ${PC_emanelte_INCLUDEDIR}
          ${PC_emanelte_INCLUDE_DIRS}
          $ENV{EMANE_MODEL_LTE_ROOT}/include
    PATHS /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    emanelte_LIBRARIES
    NAMES emanelte
    HINTS ${PC_emanelte_LIBDIR}
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          $ENV{EMANE_MODEL_LTE_ROOT}/src/.libs
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

message(STATUS "emanelte LIBRARIES " ${emanelte_LIBRARIES})
message(STATUS "emanelte INCLUDE DIRS " ${emanelte_INCLUDE_DIRS})

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(emanelte FOUND_VAR emanelte_FOUND REQUIRED_VARS emanelte_LIBRARIES emanelte_INCLUDE_DIRS)
MARK_AS_ADVANCED(emanelte_LIBRARIES emanelte_INCLUDE_DIRS)

ENDIF(NOT emanelte_FOUND)

