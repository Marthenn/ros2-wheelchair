#ifndef WHEELCHAIR__VISIBILITY_CONTROL_H_
#define WHEELCHAIR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHEELCHAIR_EXPORT __attribute__ ((dllexport))
    #define WHEELCHAIR_IMPORT __attribute__ ((dllimport))
  #else
    #define WHEELCHAIR_EXPORT __declspec(dllexport)
    #define WHEELCHAIR_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHEELCHAIR_BUILDING_DLL
    #define WHEELCHAIR_PUBLIC WHEELCHAIR_EXPORT
  #else
    #define WHEELCHAIR_PUBLIC WHEELCHAIR_IMPORT
  #endif
  #define WHEELCHAIR_PUBLIC_TYPE WHEELCHAIR_PUBLIC
  #define WHEELCHAIR_LOCAL
#else
  #define WHEELCHAIR_EXPORT __attribute__ ((visibility("default")))
  #define WHEELCHAIR_IMPORT
  #if __GNUC__ >= 4
    #define WHEELCHAIR_PUBLIC __attribute__ ((visibility("default")))
    #define WHEELCHAIR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WHEELCHAIR_PUBLIC
    #define WHEELCHAIR_LOCAL
  #endif
  #define WHEELCHAIR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // WHEELCHAIR__VISIBILITY_CONTROL_H_