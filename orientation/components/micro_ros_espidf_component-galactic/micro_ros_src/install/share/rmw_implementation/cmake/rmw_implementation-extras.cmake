# Copyright 2014-2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# generated from rmw_implementation/rmw_implementation-extras.cmake.in

find_package(rmw_implementation_cmake REQUIRED)

get_default_rmw_implementation(requested_rmw_implementation)
set(requested_rmw_implementation_from "get_default_rmw_implementation")

if(ON)
  message(STATUS "Using RMW implementation 'rmw_microxrcedds'")
  if(NOT requested_rmw_implementation STREQUAL "rmw_microxrcedds")
    message(FATAL_ERROR
      "The RMW implementation has been specified as "
      "'${requested_rmw_implementation}' via "
      "${requested_rmw_implementation_from}, but rmw_implementation was built "
      "only with support for 'rmw_microxrcedds'.")
  endif()
  find_package("rmw_microxrcedds" REQUIRED)

  # TODO should never need definitions and include dirs?
  list(APPEND rmw_implementation_DEFINITIONS
    ${rmw_microxrcedds_DEFINITIONS})
  list(APPEND rmw_implementation_INCLUDE_DIRS
    ${rmw_microxrcedds_INCLUDE_DIRS})
  list(APPEND rmw_implementation_LIBRARIES
    ${rmw_microxrcedds_LIBRARIES})
else()
  get_available_rmw_implementations(available_rmw_implementations)

  if(NOT requested_rmw_implementation IN_LIST available_rmw_implementations)
    message(FATAL_ERROR
      "The RMW implementation has been specified as "
      "'${requested_rmw_implementation}' via "
      "${requested_rmw_implementation_from}, but it is not available at this "
      "time.\n\nCurrently available middlewares:\n"
      "'${available_rmw_implementations}'")
  endif()
  message(STATUS "Using RMW implementation '${requested_rmw_implementation}' as default")

  # no need to find_package rmw_implementation
  # since this code is already part of a find_package call of that package
endif()

find_package(Threads REQUIRED)
list(APPEND rmw_implementation_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}")
