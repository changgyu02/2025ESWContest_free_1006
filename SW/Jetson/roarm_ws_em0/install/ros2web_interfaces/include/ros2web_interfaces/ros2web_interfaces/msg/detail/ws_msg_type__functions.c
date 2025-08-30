// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2web_interfaces:msg/WSMsgType.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/msg/detail/ws_msg_type__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros2web_interfaces__msg__WSMsgType__init(ros2web_interfaces__msg__WSMsgType * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ros2web_interfaces__msg__WSMsgType__fini(ros2web_interfaces__msg__WSMsgType * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ros2web_interfaces__msg__WSMsgType__are_equal(const ros2web_interfaces__msg__WSMsgType * lhs, const ros2web_interfaces__msg__WSMsgType * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ros2web_interfaces__msg__WSMsgType__copy(
  const ros2web_interfaces__msg__WSMsgType * input,
  ros2web_interfaces__msg__WSMsgType * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ros2web_interfaces__msg__WSMsgType *
ros2web_interfaces__msg__WSMsgType__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__WSMsgType * msg = (ros2web_interfaces__msg__WSMsgType *)allocator.allocate(sizeof(ros2web_interfaces__msg__WSMsgType), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2web_interfaces__msg__WSMsgType));
  bool success = ros2web_interfaces__msg__WSMsgType__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2web_interfaces__msg__WSMsgType__destroy(ros2web_interfaces__msg__WSMsgType * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2web_interfaces__msg__WSMsgType__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2web_interfaces__msg__WSMsgType__Sequence__init(ros2web_interfaces__msg__WSMsgType__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__WSMsgType * data = NULL;

  if (size) {
    data = (ros2web_interfaces__msg__WSMsgType *)allocator.zero_allocate(size, sizeof(ros2web_interfaces__msg__WSMsgType), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2web_interfaces__msg__WSMsgType__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2web_interfaces__msg__WSMsgType__fini(&data[i - 1]);
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
ros2web_interfaces__msg__WSMsgType__Sequence__fini(ros2web_interfaces__msg__WSMsgType__Sequence * array)
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
      ros2web_interfaces__msg__WSMsgType__fini(&array->data[i]);
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

ros2web_interfaces__msg__WSMsgType__Sequence *
ros2web_interfaces__msg__WSMsgType__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__WSMsgType__Sequence * array = (ros2web_interfaces__msg__WSMsgType__Sequence *)allocator.allocate(sizeof(ros2web_interfaces__msg__WSMsgType__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2web_interfaces__msg__WSMsgType__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2web_interfaces__msg__WSMsgType__Sequence__destroy(ros2web_interfaces__msg__WSMsgType__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2web_interfaces__msg__WSMsgType__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2web_interfaces__msg__WSMsgType__Sequence__are_equal(const ros2web_interfaces__msg__WSMsgType__Sequence * lhs, const ros2web_interfaces__msg__WSMsgType__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2web_interfaces__msg__WSMsgType__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2web_interfaces__msg__WSMsgType__Sequence__copy(
  const ros2web_interfaces__msg__WSMsgType__Sequence * input,
  ros2web_interfaces__msg__WSMsgType__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2web_interfaces__msg__WSMsgType);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2web_interfaces__msg__WSMsgType * data =
      (ros2web_interfaces__msg__WSMsgType *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2web_interfaces__msg__WSMsgType__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2web_interfaces__msg__WSMsgType__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2web_interfaces__msg__WSMsgType__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
