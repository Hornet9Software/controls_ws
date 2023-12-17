// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/CVObject.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__STRUCT_HPP_

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
# define DEPRECATED__custom_msgs__msg__CVObject __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__CVObject __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CVObject_
{
  using Type = CVObject_<ContainerAllocator>;

  explicit CVObject_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xmin = 0.0;
      this->ymin = 0.0;
      this->xmax = 0.0;
      this->ymax = 0.0;
      this->height = 0l;
      this->width = 0l;
      this->distance = 0.0;
      this->bearing = 0.0;
      this->leftright_ratio = 0.0;
      this->label = "";
      this->score = 0.0;
    }
  }

  explicit CVObject_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    label(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xmin = 0.0;
      this->ymin = 0.0;
      this->xmax = 0.0;
      this->ymax = 0.0;
      this->height = 0l;
      this->width = 0l;
      this->distance = 0.0;
      this->bearing = 0.0;
      this->leftright_ratio = 0.0;
      this->label = "";
      this->score = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _xmin_type =
    double;
  _xmin_type xmin;
  using _ymin_type =
    double;
  _ymin_type ymin;
  using _xmax_type =
    double;
  _xmax_type xmax;
  using _ymax_type =
    double;
  _ymax_type ymax;
  using _height_type =
    int32_t;
  _height_type height;
  using _width_type =
    int32_t;
  _width_type width;
  using _distance_type =
    double;
  _distance_type distance;
  using _bearing_type =
    double;
  _bearing_type bearing;
  using _leftright_ratio_type =
    double;
  _leftright_ratio_type leftright_ratio;
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _score_type =
    double;
  _score_type score;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__xmin(
    const double & _arg)
  {
    this->xmin = _arg;
    return *this;
  }
  Type & set__ymin(
    const double & _arg)
  {
    this->ymin = _arg;
    return *this;
  }
  Type & set__xmax(
    const double & _arg)
  {
    this->xmax = _arg;
    return *this;
  }
  Type & set__ymax(
    const double & _arg)
  {
    this->ymax = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const int32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__bearing(
    const double & _arg)
  {
    this->bearing = _arg;
    return *this;
  }
  Type & set__leftright_ratio(
    const double & _arg)
  {
    this->leftright_ratio = _arg;
    return *this;
  }
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__score(
    const double & _arg)
  {
    this->score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::CVObject_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::CVObject_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::CVObject_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::CVObject_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::CVObject_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::CVObject_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::CVObject_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::CVObject_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::CVObject_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::CVObject_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__CVObject
    std::shared_ptr<custom_msgs::msg::CVObject_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__CVObject
    std::shared_ptr<custom_msgs::msg::CVObject_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CVObject_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->xmin != other.xmin) {
      return false;
    }
    if (this->ymin != other.ymin) {
      return false;
    }
    if (this->xmax != other.xmax) {
      return false;
    }
    if (this->ymax != other.ymax) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->bearing != other.bearing) {
      return false;
    }
    if (this->leftright_ratio != other.leftright_ratio) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    return true;
  }
  bool operator!=(const CVObject_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CVObject_

// alias to use template instance with default allocator
using CVObject =
  custom_msgs::msg::CVObject_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__STRUCT_HPP_
