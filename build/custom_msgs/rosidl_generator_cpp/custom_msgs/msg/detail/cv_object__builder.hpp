// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/CVObject.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/cv_object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_CVObject_score
{
public:
  explicit Init_CVObject_score(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::CVObject score(::custom_msgs::msg::CVObject::_score_type arg)
  {
    msg_.score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_label
{
public:
  explicit Init_CVObject_label(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_score label(::custom_msgs::msg::CVObject::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_CVObject_score(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_leftright_ratio
{
public:
  explicit Init_CVObject_leftright_ratio(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_label leftright_ratio(::custom_msgs::msg::CVObject::_leftright_ratio_type arg)
  {
    msg_.leftright_ratio = std::move(arg);
    return Init_CVObject_label(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_bearing
{
public:
  explicit Init_CVObject_bearing(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_leftright_ratio bearing(::custom_msgs::msg::CVObject::_bearing_type arg)
  {
    msg_.bearing = std::move(arg);
    return Init_CVObject_leftright_ratio(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_distance
{
public:
  explicit Init_CVObject_distance(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_bearing distance(::custom_msgs::msg::CVObject::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_CVObject_bearing(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_width
{
public:
  explicit Init_CVObject_width(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_distance width(::custom_msgs::msg::CVObject::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_CVObject_distance(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_height
{
public:
  explicit Init_CVObject_height(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_width height(::custom_msgs::msg::CVObject::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_CVObject_width(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_ymax
{
public:
  explicit Init_CVObject_ymax(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_height ymax(::custom_msgs::msg::CVObject::_ymax_type arg)
  {
    msg_.ymax = std::move(arg);
    return Init_CVObject_height(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_xmax
{
public:
  explicit Init_CVObject_xmax(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_ymax xmax(::custom_msgs::msg::CVObject::_xmax_type arg)
  {
    msg_.xmax = std::move(arg);
    return Init_CVObject_ymax(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_ymin
{
public:
  explicit Init_CVObject_ymin(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_xmax ymin(::custom_msgs::msg::CVObject::_ymin_type arg)
  {
    msg_.ymin = std::move(arg);
    return Init_CVObject_xmax(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_xmin
{
public:
  explicit Init_CVObject_xmin(::custom_msgs::msg::CVObject & msg)
  : msg_(msg)
  {}
  Init_CVObject_ymin xmin(::custom_msgs::msg::CVObject::_xmin_type arg)
  {
    msg_.xmin = std::move(arg);
    return Init_CVObject_ymin(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

class Init_CVObject_header
{
public:
  Init_CVObject_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CVObject_xmin header(::custom_msgs::msg::CVObject::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CVObject_xmin(msg_);
  }

private:
  ::custom_msgs::msg::CVObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::CVObject>()
{
  return custom_msgs::msg::builder::Init_CVObject_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__BUILDER_HPP_
