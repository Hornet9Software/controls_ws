// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/CVObject.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/cv_object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `label`
#include "rosidl_runtime_c/string_functions.h"

bool
custom_msgs__msg__CVObject__init(custom_msgs__msg__CVObject * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_msgs__msg__CVObject__fini(msg);
    return false;
  }
  // xmin
  // ymin
  // xmax
  // ymax
  // height
  // width
  // distance
  // bearing
  // leftright_ratio
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    custom_msgs__msg__CVObject__fini(msg);
    return false;
  }
  // score
  return true;
}

void
custom_msgs__msg__CVObject__fini(custom_msgs__msg__CVObject * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // xmin
  // ymin
  // xmax
  // ymax
  // height
  // width
  // distance
  // bearing
  // leftright_ratio
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // score
}

bool
custom_msgs__msg__CVObject__are_equal(const custom_msgs__msg__CVObject * lhs, const custom_msgs__msg__CVObject * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // xmin
  if (lhs->xmin != rhs->xmin) {
    return false;
  }
  // ymin
  if (lhs->ymin != rhs->ymin) {
    return false;
  }
  // xmax
  if (lhs->xmax != rhs->xmax) {
    return false;
  }
  // ymax
  if (lhs->ymax != rhs->ymax) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // bearing
  if (lhs->bearing != rhs->bearing) {
    return false;
  }
  // leftright_ratio
  if (lhs->leftright_ratio != rhs->leftright_ratio) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // score
  if (lhs->score != rhs->score) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__CVObject__copy(
  const custom_msgs__msg__CVObject * input,
  custom_msgs__msg__CVObject * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // xmin
  output->xmin = input->xmin;
  // ymin
  output->ymin = input->ymin;
  // xmax
  output->xmax = input->xmax;
  // ymax
  output->ymax = input->ymax;
  // height
  output->height = input->height;
  // width
  output->width = input->width;
  // distance
  output->distance = input->distance;
  // bearing
  output->bearing = input->bearing;
  // leftright_ratio
  output->leftright_ratio = input->leftright_ratio;
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // score
  output->score = input->score;
  return true;
}

custom_msgs__msg__CVObject *
custom_msgs__msg__CVObject__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__CVObject * msg = (custom_msgs__msg__CVObject *)allocator.allocate(sizeof(custom_msgs__msg__CVObject), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__CVObject));
  bool success = custom_msgs__msg__CVObject__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__CVObject__destroy(custom_msgs__msg__CVObject * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__CVObject__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__CVObject__Sequence__init(custom_msgs__msg__CVObject__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__CVObject * data = NULL;

  if (size) {
    data = (custom_msgs__msg__CVObject *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__CVObject), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__CVObject__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__CVObject__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_msgs__msg__CVObject__Sequence__fini(custom_msgs__msg__CVObject__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__CVObject__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_msgs__msg__CVObject__Sequence *
custom_msgs__msg__CVObject__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__CVObject__Sequence * array = (custom_msgs__msg__CVObject__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__CVObject__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__CVObject__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__CVObject__Sequence__destroy(custom_msgs__msg__CVObject__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__CVObject__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__CVObject__Sequence__are_equal(const custom_msgs__msg__CVObject__Sequence * lhs, const custom_msgs__msg__CVObject__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__CVObject__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__CVObject__Sequence__copy(
  const custom_msgs__msg__CVObject__Sequence * input,
  custom_msgs__msg__CVObject__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__CVObject);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__CVObject * data =
      (custom_msgs__msg__CVObject *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__CVObject__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__CVObject__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__CVObject__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
