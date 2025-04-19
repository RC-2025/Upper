// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rc_interaction:msg/Sbus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rc_interaction/msg/detail/sbus__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rc_interaction
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Sbus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rc_interaction::msg::Sbus(_init);
}

void Sbus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rc_interaction::msg::Sbus *>(message_memory);
  typed_message->~Sbus();
}

size_t size_function__Sbus__raw_channels(const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * get_const_function__Sbus__raw_channels(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 16> *>(untyped_member);
  return &member[index];
}

void * get_function__Sbus__raw_channels(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 16> *>(untyped_member);
  return &member[index];
}

void fetch_function__Sbus__raw_channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__Sbus__raw_channels(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__Sbus__raw_channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__Sbus__raw_channels(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

size_t size_function__Sbus__mapped_channels(const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * get_const_function__Sbus__mapped_channels(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 16> *>(untyped_member);
  return &member[index];
}

void * get_function__Sbus__mapped_channels(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 16> *>(untyped_member);
  return &member[index];
}

void fetch_function__Sbus__mapped_channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__Sbus__mapped_channels(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__Sbus__mapped_channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__Sbus__mapped_channels(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Sbus_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc_interaction::msg::Sbus, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "raw_channels",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rc_interaction::msg::Sbus, raw_channels),  // bytes offset in struct
    nullptr,  // default value
    size_function__Sbus__raw_channels,  // size() function pointer
    get_const_function__Sbus__raw_channels,  // get_const(index) function pointer
    get_function__Sbus__raw_channels,  // get(index) function pointer
    fetch_function__Sbus__raw_channels,  // fetch(index, &value) function pointer
    assign_function__Sbus__raw_channels,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mapped_channels",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rc_interaction::msg::Sbus, mapped_channels),  // bytes offset in struct
    nullptr,  // default value
    size_function__Sbus__mapped_channels,  // size() function pointer
    get_const_function__Sbus__mapped_channels,  // get_const(index) function pointer
    get_function__Sbus__mapped_channels,  // get(index) function pointer
    fetch_function__Sbus__mapped_channels,  // fetch(index, &value) function pointer
    assign_function__Sbus__mapped_channels,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "failsafe",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc_interaction::msg::Sbus, failsafe),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frame_lost",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc_interaction::msg::Sbus, frame_lost),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Sbus_message_members = {
  "rc_interaction::msg",  // message namespace
  "Sbus",  // message name
  5,  // number of fields
  sizeof(rc_interaction::msg::Sbus),
  Sbus_message_member_array,  // message members
  Sbus_init_function,  // function to initialize message memory (memory has to be allocated)
  Sbus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Sbus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Sbus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rc_interaction


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rc_interaction::msg::Sbus>()
{
  return &::rc_interaction::msg::rosidl_typesupport_introspection_cpp::Sbus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rc_interaction, msg, Sbus)() {
  return &::rc_interaction::msg::rosidl_typesupport_introspection_cpp::Sbus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
