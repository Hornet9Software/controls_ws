// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ThrusterSpeeds.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/thruster_speeds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ThrusterSpeeds_speeds
{
public:
  explicit Init_ThrusterSpeeds_speeds(::custom_msgs::msg::ThrusterSpeeds & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ThrusterSpeeds speeds(::custom_msgs::msg::ThrusterSpeeds::_speeds_type arg)
  {
    msg_.speeds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ThrusterSpeeds msg_;
};

class Init_ThrusterSpeeds_header
{
public:
  Init_ThrusterSpeeds_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrusterSpeeds_speeds header(::custom_msgs::msg::ThrusterSpeeds::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ThrusterSpeeds_speeds(msg_);
  }

private:
  ::custom_msgs::msg::ThrusterSpeeds msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ThrusterSpeeds>()
{
  return custom_msgs::msg::builder::Init_ThrusterSpeeds_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__BUILDER_HPP_
