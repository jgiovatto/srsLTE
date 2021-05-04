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


# - Try to find protobuf 
# Once done this will define
#  protobuf_FOUND        - System has protobuf libraries
#  protobuf_LIBRARIES    - The protobuf library

find_package(PkgConfig)
pkg_check_modules(PC_protobuf QUIET protobuf)
IF(NOT protobuf_FOUND)

FIND_LIBRARY(
    protobuf_LIBRARIES
    NAMES protobuf
    HINTS ${PC_protobuf_LIBDIR}
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

message(STATUS "protobuf LIBRARIES " ${protobuf_LIBRARIES})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(protobuf FOUND_VAR protobuf_FOUND REQUIRED_VARS protobuf_LIBRARIES)
MARK_AS_ADVANCED(protobuf_LIBRARIES)

ENDIF(NOT protobuf_FOUND)

