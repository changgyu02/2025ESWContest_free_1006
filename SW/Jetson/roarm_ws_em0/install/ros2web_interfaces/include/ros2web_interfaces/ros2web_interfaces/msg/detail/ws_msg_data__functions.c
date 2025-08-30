// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2web_interfaces:msg/WSMsgData.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/msg/detail/ws_msg_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `str`
#include "rosidl_runtime_c/string_functions.h"
// Member `bytes`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2web_interfaces__msg__WSMsgData__init(ros2web_interfaces__msg__WSMsgData * msg)
{
  if (!msg) {
    return false;
  }
  // str
  if (!rosidl_runtime_c__String__init(&msg->str)) {
    ros2web_interfaces__msg__WSMsgData__fini(msg);
    return false;
  }
  // bytes
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->bytes, 0)) {
    ros2web_interfaces__msg__WSMsgData__fini(msg);
    return false;
  }
  return true;
}

void
ros2web_interfaces__msg__WSMsgData__fini(ros2web_interfaces__msg__WSMsgData * msg)
{
  if (!msg) {
    return;
  }
  // str
  rosidl_runtime_c__String__fini(&msg->str);
  // bytes
  rosidl_runtime_c__uint8__Sequence__fini(&msg->bytes);
}

bool
ros2web_interfaces__msg__WSMsgData__are_equal(const ros2web_interfaces__msg__WSMsgData * lhs, const ros2web_interfaces__msg__WSMsgData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // str
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->str), &(rhs->str)))
  {
    return false;
  }
  // bytes
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->bytes), &(rhs->bytes)))
  {
    return false;
  }
  return true;
}

bool
ros2web_interfaces__msg__WSMsgData__copy(
  const ros2web_interfaces__msg__WSMsgData * input,
  ros2web_interfaces__msg__WSMsgData * output)
{
  if (!input || !output) {
    return false;
  }
  // str
  if (!rosidl_runtime_c__String__copy(
      &(input->str), &(output->str)))
  {
    return false;
  }
  // bytes
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->bytes), &(output->bytes)))
  {
    return false;
  }
  return true;
}

ros2web_interfaces__msg__WSMsgData *
ros2web_interfaces__msg__WSMsgData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__WSMsgData * msg = (ros2web_interfaces__msg__WSMsgData *)allocator.allocate(sizeof(ros2web_interfaces__msg__WSMsgData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2web_interfaces__msg__WSMsgData));
  bool success = ros2web_interfaces__msg__WSMsgData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2web_interfaces__msg__WSMsgData__destroy(ros2web_interfaces__msg__WSMsgData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2web_interfaces__msg__WSMsgData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2web_interfaces__msg__WSMsgData__Sequence__init(ros2web_interfaces__msg__WSMsgData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__WSMsgData * data = NULL;

  if (size) {
    data = (ros2web_interfaces__msg__WSMsgData *)allocator.zero_allocate(size, sizeof(ros2web_interfaces__msg__WSMsgData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2web_interfaces__msg__WSMsgData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2web_interfaces__msg__WSMsgData__fini(&data[i - 1]);
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
ros2web_interfaces__msg__WSMsgData__Sequence__fini(ros2web_interfaces__msg__WSMsgData__Sequence * array)
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
      ros2web_interfaces__msg__WSMsgData__fini(&array->data[i]);
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

ros2web_interfaces__msg__WSMsgData__Sequence *
ros2web_interfaces__msg__WSMsgData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__msg__WSMsgData__Sequence * array = (ros2web_interfaces__msg__WSMsgData__Sequence *)allocator.allocate(sizeof(ros2web_interfaces__msg__WSMsgData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2web_interfaces__msg__WSMsgData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2web_interfaces__msg__WSMsgData__Sequence__destroy(ros2web_interfaces__msg__WSMsgData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2web_interfaces__msg__WSMsgData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2web_interfaces__msg__WSMsgData__Sequence__are_equal(const ros2web_interfaces__msg__WSMsgData__Sequence * lhs, const ros2web_interfaces__msg__WSMsgData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2web_interfaces__msg__WSMsgData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2web_interfaces__msg__WSMsgData__Sequence__copy(
  const ros2web_interfaces__msg__WSMsgData__Sequence * input,
  ros2web_interfaces__msg__WSMsgData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2web_interfaces__msg__WSMsgData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2web_interfaces__msg__WSMsgData * data =
      (ros2web_interfaces__msg__WSMsgData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2web_interfaces__msg__WSMsgData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2web_interfaces__msg__WSMsgData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2web_interfaces__msg__WSMsgData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
