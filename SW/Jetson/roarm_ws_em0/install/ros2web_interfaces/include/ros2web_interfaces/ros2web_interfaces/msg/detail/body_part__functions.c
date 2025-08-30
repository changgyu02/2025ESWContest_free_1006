// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2web_interfaces:msg/BodyPart.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/msg/detail/body_part__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `filename`
// Member `content_type`
// Member `headers`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2web_interfaces__msg__BodyPart__init(ros2web_interfaces__msg__BodyPart * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    ros2web_interfaces__msg__BodyPart__fini(msg);
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__init(&msg->filename)) {
    ros2web_interfaces__msg__BodyPart__fini(msg);
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__init(&msg->content_type)) {
    ros2web_interfaces__msg__BodyPart__fini(msg);
    return false;
  }
  // headers
  if (!rosidl_runtime_c__String__Sequence__init(&msg->headers, 0)) {
    ros2web_interfaces__msg__BodyPart__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    ros2web_interfaces__msg__BodyPart__fini(msg);
    return false;
  }
  return true;
}

void
ros2web_interfaces__msg__BodyPart__fini(ros2web_interfaces__msg__BodyPart * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // filename
  rosidl_runtime_c__String__fini(&msg->filename);
  // content_type
  rosidl_runtime_c__String__fini(&msg->content_type);
  // headers
  rosidl_runtime_c__String__Sequence__fini(&msg->headers);
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
ros2web_interfaces__msg__BodyPart__are_equal(const ros2web_interfaces__msg__BodyPart * lhs, const ros2web_interfaces__msg__BodyPart * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->filename), &(rhs->filename)))
  {
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->content_type), &(rhs->content_type)))
  {
    return false;
  }
  // headers
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->headers), &(rhs->headers)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
ros2web_interfaces__msg__BodyPart__copy(
  const ros2web_interfaces__msg__BodyPart * input,
  ros2web_interfaces__msg__BodyPart * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__copy(
      &(input->filename), &(output->filename)))
  {
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__copy(
      &(input->content_type), &(output->content_type)))
  {
    return false;
  }
  // headers
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->headers), &(output->headers)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

ros2web_interfaces__msg__BodyPart *
ros2web_interfaces__msg__BodyPart__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__BodyPart * msg = (ros2web_interfaces__msg__BodyPart *)allocator.allocate(sizeof(ros2web_interfaces__msg__BodyPart), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2web_interfaces__msg__BodyPart));
  bool success = ros2web_interfaces__msg__BodyPart__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2web_interfaces__msg__BodyPart__destroy(ros2web_interfaces__msg__BodyPart * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2web_interfaces__msg__BodyPart__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2web_interfaces__msg__BodyPart__Sequence__init(ros2web_interfaces__msg__BodyPart__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__BodyPart * data = NULL;

  if (size) {
    data = (ros2web_interfaces__msg__BodyPart *)allocator.zero_allocate(size, sizeof(ros2web_interfaces__msg__BodyPart), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2web_interfaces__msg__BodyPart__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2web_interfaces__msg__BodyPart__fini(&data[i - 1]);
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
ros2web_interfaces__msg__BodyPart__Sequence__fini(ros2web_interfaces__msg__BodyPart__Sequence * array)
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
      ros2web_interfaces__msg__BodyPart__fini(&array->data[i]);
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

ros2web_interfaces__msg__BodyPart__Sequence *
ros2web_interfaces__msg__BodyPart__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__BodyPart__Sequence * array = (ros2web_interfaces__msg__BodyPart__Sequence *)allocator.allocate(sizeof(ros2web_interfaces__msg__BodyPart__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2web_interfaces__msg__BodyPart__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2web_interfaces__msg__BodyPart__Sequence__destroy(ros2web_interfaces__msg__BodyPart__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2web_interfaces__msg__BodyPart__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2web_interfaces__msg__BodyPart__Sequence__are_equal(const ros2web_interfaces__msg__BodyPart__Sequence * lhs, const ros2web_interfaces__msg__BodyPart__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2web_interfaces__msg__BodyPart__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2web_interfaces__msg__BodyPart__Sequence__copy(
  const ros2web_interfaces__msg__BodyPart__Sequence * input,
  ros2web_interfaces__msg__BodyPart__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2web_interfaces__msg__BodyPart);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2web_interfaces__msg__BodyPart * data =
      (ros2web_interfaces__msg__BodyPart *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2web_interfaces__msg__BodyPart__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2web_interfaces__msg__BodyPart__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2web_interfaces__msg__BodyPart__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
