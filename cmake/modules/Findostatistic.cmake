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


# - Try to find ostatistic 
# Once done this will define
#  ostatistic_FOUND        - System has ostatistic
#  ostatistic_LIBRARIES    - ostatistic library


find_package(PkgConfig)
pkg_check_modules(PC_ostatistic QUIET libostatistic)
IF(NOT ostatistic_FOUND)











FIND_LIBRARY(
    ostatistic_LIBRARIES
    NAMES ostatistic
    HINTS ${PC_ostatistic_LIBDIR}
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          $ENV{OSTATISTIC_ROOT}/src/libostatistic/.libs
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

message(STATUS "ostatistic LIBRARIES " ${ostatistic_LIBRARIES})


INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(ostatistic FOUND_VAR ostatistic_FOUND REQUIRED_VARS ostatistic_LIBRARIES)
MARK_AS_ADVANCED(ostatistic_LIBRARIES)

ENDIF(NOT ostatistic_FOUND)

