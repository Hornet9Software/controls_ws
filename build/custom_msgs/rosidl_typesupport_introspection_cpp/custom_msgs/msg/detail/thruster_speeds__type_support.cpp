// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from custom_msgs:msg/ThrusterSpeeds.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_msgs/msg/detail/thruster_speeds__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace custom_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ThrusterSpeeds_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) custom_msgs::msg::ThrusterSpeeds(_init);
}

void ThrusterSpeeds_fini_function(void * message_memory)
{
  auto typed_message = static_cast<custom_msgs::msg::ThrusterSpeeds *>(message_memory);
  typed_message->~ThrusterSpeeds();
}

size_t size_function__ThrusterSpeeds__speeds(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__ThrusterSpeeds__speeds(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int8_t, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__ThrusterSpeeds__speeds(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int8_t, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__ThrusterSpeeds__speeds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__ThrusterSpeeds__speeds(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__ThrusterSpeeds__speeds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__ThrusterSpeeds__speeds(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ThrusterSpeeds_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::ThrusterSpeeds, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speeds",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::ThrusterSpeeds, speeds),  // bytes offset in struct
    nullptr,  // default value
    size_function__ThrusterSpeeds__speeds,  // size() function pointer
    get_const_function__ThrusterSpeeds__speeds,  // get_const(index) function pointer
    get_function__ThrusterSpeeds__speeds,  // get(index) function pointer
    fetch_function__ThrusterSpeeds__speeds,  // fetch(index, &value) function pointer
    assign_function__ThrusterSpeeds__speeds,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ThrusterSpeeds_message_members = {
  "custom_msgs::msg",  // message namespace
  "ThrusterSpeeds",  // message name
  2,  // number of fields
  sizeof(custom_msgs::msg::ThrusterSpeeds),
  ThrusterSpeeds_message_member_array,  // message members
  ThrusterSpeeds_init_function,  // function to initialize message memory (memory has to be allocated)
  ThrusterSpeeds_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ThrusterSpeeds_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ThrusterSpeeds_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace custom_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_msgs::msg::ThrusterSpeeds>()
{
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::ThrusterSpeeds_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_msgs, msg, ThrusterSpeeds)() {
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::ThrusterSpeeds_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
