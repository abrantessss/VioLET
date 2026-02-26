#ifndef VIOLET_MSGS_CPP__VISIBILITY_CONTROL_H_
#define VIOLET_MSGS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIOLET_MSGS_CPP_EXPORT __attribute__ ((dllexport))
    #define VIOLET_MSGS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define VIOLET_MSGS_CPP_EXPORT __declspec(dllexport)
    #define VIOLET_MSGS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIOLET_MSGS_CPP_BUILDING_DLL
    #define VIOLET_MSGS_CPP_PUBLIC VIOLET_MSGS_CPP_EXPORT
  #else
    #define VIOLET_MSGS_CPP_PUBLIC VIOLET_MSGS_CPP_IMPORT
  #endif
  #define VIOLET_MSGS_CPP_PUBLIC_TYPE VIOLET_MSGS_CPP_PUBLIC
  #define VIOLET_MSGS_CPP_LOCAL
#else
  #define VIOLET_MSGS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define VIOLET_MSGS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define VIOLET_MSGS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define VIOLET_MSGS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIOLET_MSGS_CPP_PUBLIC
    #define VIOLET_MSGS_CPP_LOCAL
  #endif
  #define VIOLET_MSGS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // VIOLET_MSGS_CPP__VISIBILITY_CONTROL_H_