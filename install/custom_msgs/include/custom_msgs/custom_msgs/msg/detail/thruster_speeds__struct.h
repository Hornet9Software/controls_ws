// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/ThrusterSpeeds.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ThrusterSpeeds in the package custom_msgs.
/**
  * Message containing thruster speeds
 */
typedef struct custom_msgs__msg__ThrusterSpeeds
{
  std_msgs__msg__Header header;
  /// Array of thruster speeds
  int8_t speeds[6];
} custom_msgs__msg__ThrusterSpeeds;

// Struct for a sequence of custom_msgs__msg__ThrusterSpeeds.
typedef struct custom_msgs__msg__ThrusterSpeeds__Sequence
{
  custom_msgs__msg__ThrusterSpeeds * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__ThrusterSpeeds__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__THRUSTER_SPEEDS__STRUCT_H_
