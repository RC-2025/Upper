// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc_interaction:msg/Sbus.idl
// generated code does not contain a copyright notice

#ifndef RC_INTERACTION__MSG__DETAIL__SBUS__BUILDER_HPP_
#define RC_INTERACTION__MSG__DETAIL__SBUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rc_interaction/msg/detail/sbus__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rc_interaction
{

namespace msg
{

namespace builder
{

class Init_Sbus_frame_lost
{
public:
  explicit Init_Sbus_frame_lost(::rc_interaction::msg::Sbus & msg)
  : msg_(msg)
  {}
  ::rc_interaction::msg::Sbus frame_lost(::rc_interaction::msg::Sbus::_frame_lost_type arg)
  {
    msg_.frame_lost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc_interaction::msg::Sbus msg_;
};

class Init_Sbus_failsafe
{
public:
  explicit Init_Sbus_failsafe(::rc_interaction::msg::Sbus & msg)
  : msg_(msg)
  {}
  Init_Sbus_frame_lost failsafe(::rc_interaction::msg::Sbus::_failsafe_type arg)
  {
    msg_.failsafe = std::move(arg);
    return Init_Sbus_frame_lost(msg_);
  }

private:
  ::rc_interaction::msg::Sbus msg_;
};

class Init_Sbus_mapped_channels
{
public:
  explicit Init_Sbus_mapped_channels(::rc_interaction::msg::Sbus & msg)
  : msg_(msg)
  {}
  Init_Sbus_failsafe mapped_channels(::rc_interaction::msg::Sbus::_mapped_channels_type arg)
  {
    msg_.mapped_channels = std::move(arg);
    return Init_Sbus_failsafe(msg_);
  }

private:
  ::rc_interaction::msg::Sbus msg_;
};

class Init_Sbus_raw_channels
{
public:
  explicit Init_Sbus_raw_channels(::rc_interaction::msg::Sbus & msg)
  : msg_(msg)
  {}
  Init_Sbus_mapped_channels raw_channels(::rc_interaction::msg::Sbus::_raw_channels_type arg)
  {
    msg_.raw_channels = std::move(arg);
    return Init_Sbus_mapped_channels(msg_);
  }

private:
  ::rc_interaction::msg::Sbus msg_;
};

class Init_Sbus_header
{
public:
  Init_Sbus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sbus_raw_channels header(::rc_interaction::msg::Sbus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Sbus_raw_channels(msg_);
  }

private:
  ::rc_interaction::msg::Sbus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc_interaction::msg::Sbus>()
{
  return rc_interaction::msg::builder::Init_Sbus_header();
}

}  // namespace rc_interaction

#endif  // RC_INTERACTION__MSG__DETAIL__SBUS__BUILDER_HPP_
