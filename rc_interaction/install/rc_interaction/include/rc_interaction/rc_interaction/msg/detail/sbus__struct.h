// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc_interaction:msg/Sbus.idl
// generated code does not contain a copyright notice

#ifndef RC_INTERACTION__MSG__DETAIL__SBUS__STRUCT_H_
#define RC_INTERACTION__MSG__DETAIL__SBUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Sbus in the package rc_interaction.
typedef struct rc_interaction__msg__Sbus
{
  std_msgs__msg__Header header;
  int16_t raw_channels[16];
  int16_t mapped_channels[16];
  bool failsafe;
  bool frame_lost;
} rc_interaction__msg__Sbus;

// Struct for a sequence of rc_interaction__msg__Sbus.
typedef struct rc_interaction__msg__Sbus__Sequence
{
  rc_interaction__msg__Sbus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc_interaction__msg__Sbus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC_INTERACTION__MSG__DETAIL__SBUS__STRUCT_H_
