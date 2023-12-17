// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/CVObject.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__STRUCT_H_

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
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CVObject in the package custom_msgs.
typedef struct custom_msgs__msg__CVObject
{
  std_msgs__msg__Header header;
  /// max/min coordinates for bounding boxes
  double xmin;
  double ymin;
  double xmax;
  double ymax;
  int32_t height;
  int32_t width;
  double distance;
  double bearing;
  double leftright_ratio;
  rosidl_runtime_c__String label;
  double score;
} custom_msgs__msg__CVObject;

// Struct for a sequence of custom_msgs__msg__CVObject.
typedef struct custom_msgs__msg__CVObject__Sequence
{
  custom_msgs__msg__CVObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__CVObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__STRUCT_H_
