// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rc_interaction:msg/Sbus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rc_interaction/msg/detail/sbus__rosidl_typesupport_introspection_c.h"
#include "rc_interaction/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rc_interaction/msg/detail/sbus__functions.h"
#include "rc_interaction/msg/detail/sbus__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc_interaction__msg__Sbus__init(message_memory);
}

void rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_fini_function(void * message_memory)
{
  rc_interaction__msg__Sbus__fini(message_memory);
}

size_t rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__size_function__Sbus__raw_channels(
  const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_const_function__Sbus__raw_channels(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_function__Sbus__raw_channels(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__fetch_function__Sbus__raw_channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_const_function__Sbus__raw_channels(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__assign_function__Sbus__raw_channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_function__Sbus__raw_channels(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__size_function__Sbus__mapped_channels(
  const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_const_function__Sbus__mapped_channels(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_function__Sbus__mapped_channels(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__fetch_function__Sbus__mapped_channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_const_function__Sbus__mapped_channels(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__assign_function__Sbus__mapped_channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_function__Sbus__mapped_channels(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc_interaction__msg__Sbus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "raw_channels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rc_interaction__msg__Sbus, raw_channels),  // bytes offset in struct
    NULL,  // default value
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__size_function__Sbus__raw_channels,  // size() function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_const_function__Sbus__raw_channels,  // get_const(index) function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_function__Sbus__raw_channels,  // get(index) function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__fetch_function__Sbus__raw_channels,  // fetch(index, &value) function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__assign_function__Sbus__raw_channels,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mapped_channels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rc_interaction__msg__Sbus, mapped_channels),  // bytes offset in struct
    NULL,  // default value
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__size_function__Sbus__mapped_channels,  // size() function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_const_function__Sbus__mapped_channels,  // get_const(index) function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__get_function__Sbus__mapped_channels,  // get(index) function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__fetch_function__Sbus__mapped_channels,  // fetch(index, &value) function pointer
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__assign_function__Sbus__mapped_channels,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failsafe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc_interaction__msg__Sbus, failsafe),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame_lost",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc_interaction__msg__Sbus, frame_lost),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_members = {
  "rc_interaction__msg",  // message namespace
  "Sbus",  // message name
  5,  // number of fields
  sizeof(rc_interaction__msg__Sbus),
  rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_member_array,  // message members
  rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_init_function,  // function to initialize message memory (memory has to be allocated)
  rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_type_support_handle = {
  0,
  &rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc_interaction
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc_interaction, msg, Sbus)() {
  rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_type_support_handle.typesupport_identifier) {
    rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rc_interaction__msg__Sbus__rosidl_typesupport_introspection_c__Sbus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
