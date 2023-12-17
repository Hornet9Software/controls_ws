// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_msgs:msg/CVObject.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__FUNCTIONS_H_
#define CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "custom_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "custom_msgs/msg/detail/cv_object__struct.h"

/// Initialize msg/CVObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_msgs__msg__CVObject
 * )) before or use
 * custom_msgs__msg__CVObject__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
bool
custom_msgs__msg__CVObject__init(custom_msgs__msg__CVObject * msg);

/// Finalize msg/CVObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
void
custom_msgs__msg__CVObject__fini(custom_msgs__msg__CVObject * msg);

/// Create msg/CVObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_msgs__msg__CVObject__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
custom_msgs__msg__CVObject *
custom_msgs__msg__CVObject__create();

/// Destroy msg/CVObject message.
/**
 * It calls
 * custom_msgs__msg__CVObject__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
void
custom_msgs__msg__CVObject__destroy(custom_msgs__msg__CVObject * msg);

/// Check for msg/CVObject message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
bool
custom_msgs__msg__CVObject__are_equal(const custom_msgs__msg__CVObject * lhs, const custom_msgs__msg__CVObject * rhs);

/// Copy a msg/CVObject message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
bool
custom_msgs__msg__CVObject__copy(
  const custom_msgs__msg__CVObject * input,
  custom_msgs__msg__CVObject * output);

/// Initialize array of msg/CVObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_msgs__msg__CVObject__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
bool
custom_msgs__msg__CVObject__Sequence__init(custom_msgs__msg__CVObject__Sequence * array, size_t size);

/// Finalize array of msg/CVObject messages.
/**
 * It calls
 * custom_msgs__msg__CVObject__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
void
custom_msgs__msg__CVObject__Sequence__fini(custom_msgs__msg__CVObject__Sequence * array);

/// Create array of msg/CVObject messages.
/**
 * It allocates the memory for the array and calls
 * custom_msgs__msg__CVObject__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
custom_msgs__msg__CVObject__Sequence *
custom_msgs__msg__CVObject__Sequence__create(size_t size);

/// Destroy array of msg/CVObject messages.
/**
 * It calls
 * custom_msgs__msg__CVObject__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
void
custom_msgs__msg__CVObject__Sequence__destroy(custom_msgs__msg__CVObject__Sequence * array);

/// Check for msg/CVObject message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
bool
custom_msgs__msg__CVObject__Sequence__are_equal(const custom_msgs__msg__CVObject__Sequence * lhs, const custom_msgs__msg__CVObject__Sequence * rhs);

/// Copy an array of msg/CVObject messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
bool
custom_msgs__msg__CVObject__Sequence__copy(
  const custom_msgs__msg__CVObject__Sequence * input,
  custom_msgs__msg__CVObject__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__CV_OBJECT__FUNCTIONS_H_
