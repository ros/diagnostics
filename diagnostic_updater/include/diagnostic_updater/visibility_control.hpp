#ifndef DIAGNOSTIC_UPDATER__VISIBILITY_CONTROL_HPP_
#define DIAGNOSTIC_UPDATER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DIAGNOSTIC_UPDATER_EXPORT __attribute__ ((dllexport))
    #define DIAGNOSTIC_UPDATER_IMPORT __attribute__ ((dllimport))
  #else
    #define DIAGNOSTIC_UPDATER_EXPORT __declspec(dllexport)
    #define DIAGNOSTIC_UPDATER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DIAGNOSTIC_UPDATER_BUILDING_DLL
    #define DIAGNOSTIC_UPDATER_PUBLIC DIAGNOSTIC_UPDATER_EXPORT
  #else
    #define DIAGNOSTIC_UPDATER_PUBLIC DIAGNOSTIC_UPDATER_IMPORT
  #endif
  #define DIAGNOSTIC_UPDATER_PUBLIC_TYPE DIAGNOSTIC_UPDATER_PUBLIC
  #define DIAGNOSTIC_UPDATER_LOCAL
#else
  #define DIAGNOSTIC_UPDATER_EXPORT __attribute__ ((visibility ("default")))
  #define DIAGNOSTIC_UPDATER_IMPORT
  #define DIAGNOSTIC_UPDATER_PUBLIC __attribute__ ((visibility ("default")))
  #define DIAGNOSTIC_UPDATER_LOCAL  __attribute__ ((visibility ("hidden")))
  #define DIAGNOSTIC_UPDATER_PUBLIC_TYPE
#endif

#endif  // DIAGNOSTIC_UPDATER__VISIBILITY_CONTROL_HPP_

