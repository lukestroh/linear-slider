#ifndef __LINEAR_SLIDER_CONTROLLER__VISIBILITY_CONTROL_H__
#define __LINEAR_SLIDER_CONTROLLER__VISIBILITY_CONTROL_H__

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LINEAR_SLIDER_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define LINEAR_SLIDER_CONTROLLER_IMPORT  __attribute__ ((unused))
  #else
    #define LINEAR_SLIDER_CONTROLLER_EXPORT __declspec(dllexport)
    #define LINEAR_SLIDER_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
    #ifdef LINEAR_SLIDER_CONTROLLER_BUILDING_DLL
        #define LINEAR_SLIDER_CONTROLLER_PUBLIC LINEAR_SLIDER_CONTROLLER_EXPORT
    #else
        #define LINEAR_SLIDER_CONTROLLER_PUBLIC LINEAR_SLIDER_CONTROLLER_IMPORT
    #endif
    #define LINEAR_SLIDER_CONTROLLER_PUBLIC_TYPE LINEAR_SLIDER_CONTROLLER_PUBLIC
    #define LINEAR_SLIDER_CONTROLLER_LOCAL
#else
    #define LINEAR_SLIDER_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
    #define LINEAR_SLIDER_CONTROLLER_IMPORT
    #if __GNUC__ >= 4
        #define LINEAR_SLIDER_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
        #define LINEAR_SLIDER_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define LINEAR_SLIDER_CONTROLLER_PUBLIC
        #define LINEAR_SLIDER_CONTROLLER_LOCAL
    #endif
    #define LINEAR_SLIDER_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // __LINEAR_SLIDER_CONTROLLER__VISIBILITY_CONTROL_H__