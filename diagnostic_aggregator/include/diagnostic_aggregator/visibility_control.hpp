// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIAGNOSTIC_AGGREGATOR__VISIBILITY_CONTROL_HPP_
#define DIAGNOSTIC_AGGREGATOR__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DIAGNOSTIC_AGGREGATOR_EXPORT __attribute__ ((dllexport))
    #define DIAGNOSTIC_AGGREGATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define DIAGNOSTIC_AGGREGATOR_EXPORT __declspec(dllexport)
    #define DIAGNOSTIC_AGGREGATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef DIAGNOSTIC_AGGREGATOR_BUILDING_DLL
    #define DIAGNOSTIC_AGGREGATOR_PUBLIC DIAGNOSTIC_AGGREGATOR_EXPORT
  #else
    #define DIAGNOSTIC_AGGREGATOR_PUBLIC DIAGNOSTIC_AGGREGATOR_IMPORT
  #endif
  #define DIAGNOSTIC_AGGREGATOR_PUBLIC_TYPE DIAGNOSTIC_AGGREGATOR_PUBLIC
  #define DIAGNOSTIC_AGGREGATOR_LOCAL
#else
  #define DIAGNOSTIC_AGGREGATOR_EXPORT __attribute__ ((visibility("default")))
  #define DIAGNOSTIC_AGGREGATOR_IMPORT
  #if __GNUC__ >= 4
    #define DIAGNOSTIC_AGGREGATOR_PUBLIC __attribute__ ((visibility("default")))
    #define DIAGNOSTIC_AGGREGATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DIAGNOSTIC_AGGREGATOR_PUBLIC
    #define DIAGNOSTIC_AGGREGATOR_LOCAL
  #endif
  #define DIAGNOSTIC_AGGREGATOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DIAGNOSTIC_AGGREGATOR__VISIBILITY_CONTROL_HPP_
