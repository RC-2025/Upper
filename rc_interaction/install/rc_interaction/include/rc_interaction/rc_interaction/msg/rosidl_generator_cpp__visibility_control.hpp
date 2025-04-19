// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef RC_INTERACTION__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define RC_INTERACTION__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_rc_interaction __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_rc_interaction __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_rc_interaction __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_rc_interaction __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_rc_interaction
    #define ROSIDL_GENERATOR_CPP_PUBLIC_rc_interaction ROSIDL_GENERATOR_CPP_EXPORT_rc_interaction
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_rc_interaction ROSIDL_GENERATOR_CPP_IMPORT_rc_interaction
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_rc_interaction __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_rc_interaction
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_rc_interaction __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_rc_interaction
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // RC_INTERACTION__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
