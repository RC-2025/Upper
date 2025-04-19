// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc_interaction:msg/Sbus.idl
// generated code does not contain a copyright notice

#ifndef RC_INTERACTION__MSG__DETAIL__SBUS__STRUCT_HPP_
#define RC_INTERACTION__MSG__DETAIL__SBUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rc_interaction__msg__Sbus __attribute__((deprecated))
#else
# define DEPRECATED__rc_interaction__msg__Sbus __declspec(deprecated)
#endif

namespace rc_interaction
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sbus_
{
  using Type = Sbus_<ContainerAllocator>;

  explicit Sbus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int16_t, 16>::iterator, int16_t>(this->raw_channels.begin(), this->raw_channels.end(), 0);
      std::fill<typename std::array<int16_t, 16>::iterator, int16_t>(this->mapped_channels.begin(), this->mapped_channels.end(), 0);
      this->failsafe = false;
      this->frame_lost = false;
    }
  }

  explicit Sbus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    raw_channels(_alloc),
    mapped_channels(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int16_t, 16>::iterator, int16_t>(this->raw_channels.begin(), this->raw_channels.end(), 0);
      std::fill<typename std::array<int16_t, 16>::iterator, int16_t>(this->mapped_channels.begin(), this->mapped_channels.end(), 0);
      this->failsafe = false;
      this->frame_lost = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _raw_channels_type =
    std::array<int16_t, 16>;
  _raw_channels_type raw_channels;
  using _mapped_channels_type =
    std::array<int16_t, 16>;
  _mapped_channels_type mapped_channels;
  using _failsafe_type =
    bool;
  _failsafe_type failsafe;
  using _frame_lost_type =
    bool;
  _frame_lost_type frame_lost;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__raw_channels(
    const std::array<int16_t, 16> & _arg)
  {
    this->raw_channels = _arg;
    return *this;
  }
  Type & set__mapped_channels(
    const std::array<int16_t, 16> & _arg)
  {
    this->mapped_channels = _arg;
    return *this;
  }
  Type & set__failsafe(
    const bool & _arg)
  {
    this->failsafe = _arg;
    return *this;
  }
  Type & set__frame_lost(
    const bool & _arg)
  {
    this->frame_lost = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc_interaction::msg::Sbus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc_interaction::msg::Sbus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc_interaction::msg::Sbus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc_interaction::msg::Sbus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc_interaction::msg::Sbus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc_interaction::msg::Sbus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc_interaction::msg::Sbus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc_interaction::msg::Sbus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc_interaction::msg::Sbus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc_interaction::msg::Sbus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc_interaction__msg__Sbus
    std::shared_ptr<rc_interaction::msg::Sbus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc_interaction__msg__Sbus
    std::shared_ptr<rc_interaction::msg::Sbus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sbus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->raw_channels != other.raw_channels) {
      return false;
    }
    if (this->mapped_channels != other.mapped_channels) {
      return false;
    }
    if (this->failsafe != other.failsafe) {
      return false;
    }
    if (this->frame_lost != other.frame_lost) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sbus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sbus_

// alias to use template instance with default allocator
using Sbus =
  rc_interaction::msg::Sbus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc_interaction

#endif  // RC_INTERACTION__MSG__DETAIL__SBUS__STRUCT_HPP_
