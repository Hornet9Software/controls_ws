// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/ThrusterSpeeds.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__STRUCT_HPP_

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
# define DEPRECATED__custom_msgs__msg__ThrusterSpeeds __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__ThrusterSpeeds __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ThrusterSpeeds_
{
  using Type = ThrusterSpeeds_<ContainerAllocator>;

  explicit ThrusterSpeeds_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int8_t, 6>::iterator, int8_t>(this->speeds.begin(), this->speeds.end(), 0);
    }
  }

  explicit ThrusterSpeeds_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    speeds(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int8_t, 6>::iterator, int8_t>(this->speeds.begin(), this->speeds.end(), 0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _speeds_type =
    std::array<int8_t, 6>;
  _speeds_type speeds;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__speeds(
    const std::array<int8_t, 6> & _arg)
  {
    this->speeds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__ThrusterSpeeds
    std::shared_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__ThrusterSpeeds
    std::shared_ptr<custom_msgs::msg::ThrusterSpeeds_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ThrusterSpeeds_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->speeds != other.speeds) {
      return false;
    }
    return true;
  }
  bool operator!=(const ThrusterSpeeds_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ThrusterSpeeds_

// alias to use template instance with default allocator
using ThrusterSpeeds =
  custom_msgs::msg::ThrusterSpeeds_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__STRUCT_HPP_
