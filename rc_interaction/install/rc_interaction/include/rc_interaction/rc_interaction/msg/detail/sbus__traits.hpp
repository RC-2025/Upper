// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc_interaction:msg/Sbus.idl
// generated code does not contain a copyright notice

#ifndef RC_INTERACTION__MSG__DETAIL__SBUS__TRAITS_HPP_
#define RC_INTERACTION__MSG__DETAIL__SBUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rc_interaction/msg/detail/sbus__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rc_interaction
{

namespace msg
{

inline void to_flow_style_yaml(
  const Sbus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: raw_channels
  {
    if (msg.raw_channels.size() == 0) {
      out << "raw_channels: []";
    } else {
      out << "raw_channels: [";
      size_t pending_items = msg.raw_channels.size();
      for (auto item : msg.raw_channels) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mapped_channels
  {
    if (msg.mapped_channels.size() == 0) {
      out << "mapped_channels: []";
    } else {
      out << "mapped_channels: [";
      size_t pending_items = msg.mapped_channels.size();
      for (auto item : msg.mapped_channels) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: failsafe
  {
    out << "failsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe, out);
    out << ", ";
  }

  // member: frame_lost
  {
    out << "frame_lost: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_lost, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sbus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: raw_channels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.raw_channels.size() == 0) {
      out << "raw_channels: []\n";
    } else {
      out << "raw_channels:\n";
      for (auto item : msg.raw_channels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mapped_channels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.mapped_channels.size() == 0) {
      out << "mapped_channels: []\n";
    } else {
      out << "mapped_channels:\n";
      for (auto item : msg.mapped_channels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: failsafe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe, out);
    out << "\n";
  }

  // member: frame_lost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_lost: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_lost, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sbus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rc_interaction

namespace rosidl_generator_traits
{

[[deprecated("use rc_interaction::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rc_interaction::msg::Sbus & msg,
  std::ostream & out, size_t indentation = 0)
{
  rc_interaction::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rc_interaction::msg::to_yaml() instead")]]
inline std::string to_yaml(const rc_interaction::msg::Sbus & msg)
{
  return rc_interaction::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rc_interaction::msg::Sbus>()
{
  return "rc_interaction::msg::Sbus";
}

template<>
inline const char * name<rc_interaction::msg::Sbus>()
{
  return "rc_interaction/msg/Sbus";
}

template<>
struct has_fixed_size<rc_interaction::msg::Sbus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rc_interaction::msg::Sbus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rc_interaction::msg::Sbus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC_INTERACTION__MSG__DETAIL__SBUS__TRAITS_HPP_
