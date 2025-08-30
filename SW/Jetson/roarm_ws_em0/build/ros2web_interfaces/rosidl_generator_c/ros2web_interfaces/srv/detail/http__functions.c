// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/srv/detail/http__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `headers`
// Member `method`
// Member `path`
// Member `query`
// Member `srv_name`
// Member `route`
// Member `text`
// Member `content_type`
// Member `charset`
#include "rosidl_runtime_c/string_functions.h"
// Member `body`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `multipart`
#include "ros2web_interfaces/msg/detail/body_part__functions.h"

bool
ros2web_interfaces__srv__HTTP_Request__init(ros2web_interfaces__srv__HTTP_Request * msg)
{
  if (!msg) {
    return false;
  }
  // headers
  if (!rosidl_runtime_c__String__Sequence__init(&msg->headers, 0)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // method
  if (!rosidl_runtime_c__String__init(&msg->method)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__init(&msg->path)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // query
  if (!rosidl_runtime_c__String__init(&msg->query)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // srv_name
  if (!rosidl_runtime_c__String__init(&msg->srv_name)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // route
  if (!rosidl_runtime_c__String__init(&msg->route)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // body
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->body, 0)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__init(&msg->content_type)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // charset
  if (!rosidl_runtime_c__String__init(&msg->charset)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  // multipart
  if (!ros2web_interfaces__msg__BodyPart__Sequence__init(&msg->multipart, 0)) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
    return false;
  }
  return true;
}

void
ros2web_interfaces__srv__HTTP_Request__fini(ros2web_interfaces__srv__HTTP_Request * msg)
{
  if (!msg) {
    return;
  }
  // headers
  rosidl_runtime_c__String__Sequence__fini(&msg->headers);
  // method
  rosidl_runtime_c__String__fini(&msg->method);
  // path
  rosidl_runtime_c__String__fini(&msg->path);
  // query
  rosidl_runtime_c__String__fini(&msg->query);
  // srv_name
  rosidl_runtime_c__String__fini(&msg->srv_name);
  // route
  rosidl_runtime_c__String__fini(&msg->route);
  // body
  rosidl_runtime_c__uint8__Sequence__fini(&msg->body);
  // text
  rosidl_runtime_c__String__fini(&msg->text);
  // content_type
  rosidl_runtime_c__String__fini(&msg->content_type);
  // charset
  rosidl_runtime_c__String__fini(&msg->charset);
  // multipart
  ros2web_interfaces__msg__BodyPart__Sequence__fini(&msg->multipart);
}

bool
ros2web_interfaces__srv__HTTP_Request__are_equal(const ros2web_interfaces__srv__HTTP_Request * lhs, const ros2web_interfaces__srv__HTTP_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // headers
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->headers), &(rhs->headers)))
  {
    return false;
  }
  // method
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->method), &(rhs->method)))
  {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  // query
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->query), &(rhs->query)))
  {
    return false;
  }
  // srv_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->srv_name), &(rhs->srv_name)))
  {
    return false;
  }
  // route
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->route), &(rhs->route)))
  {
    return false;
  }
  // body
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->body), &(rhs->body)))
  {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->content_type), &(rhs->content_type)))
  {
    return false;
  }
  // charset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->charset), &(rhs->charset)))
  {
    return false;
  }
  // multipart
  if (!ros2web_interfaces__msg__BodyPart__Sequence__are_equal(
      &(lhs->multipart), &(rhs->multipart)))
  {
    return false;
  }
  return true;
}

bool
ros2web_interfaces__srv__HTTP_Request__copy(
  const ros2web_interfaces__srv__HTTP_Request * input,
  ros2web_interfaces__srv__HTTP_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // headers
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->headers), &(output->headers)))
  {
    return false;
  }
  // method
  if (!rosidl_runtime_c__String__copy(
      &(input->method), &(output->method)))
  {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  // query
  if (!rosidl_runtime_c__String__copy(
      &(input->query), &(output->query)))
  {
    return false;
  }
  // srv_name
  if (!rosidl_runtime_c__String__copy(
      &(input->srv_name), &(output->srv_name)))
  {
    return false;
  }
  // route
  if (!rosidl_runtime_c__String__copy(
      &(input->route), &(output->route)))
  {
    return false;
  }
  // body
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->body), &(output->body)))
  {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__copy(
      &(input->content_type), &(output->content_type)))
  {
    return false;
  }
  // charset
  if (!rosidl_runtime_c__String__copy(
      &(input->charset), &(output->charset)))
  {
    return false;
  }
  // multipart
  if (!ros2web_interfaces__msg__BodyPart__Sequence__copy(
      &(input->multipart), &(output->multipart)))
  {
    return false;
  }
  return true;
}

ros2web_interfaces__srv__HTTP_Request *
ros2web_interfaces__srv__HTTP_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__srv__HTTP_Request * msg = (ros2web_interfaces__srv__HTTP_Request *)allocator.allocate(sizeof(ros2web_interfaces__srv__HTTP_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2web_interfaces__srv__HTTP_Request));
  bool success = ros2web_interfaces__srv__HTTP_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2web_interfaces__srv__HTTP_Request__destroy(ros2web_interfaces__srv__HTTP_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2web_interfaces__srv__HTTP_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2web_interfaces__srv__HTTP_Request__Sequence__init(ros2web_interfaces__srv__HTTP_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__srv__HTTP_Request * data = NULL;

  if (size) {
    data = (ros2web_interfaces__srv__HTTP_Request *)allocator.zero_allocate(size, sizeof(ros2web_interfaces__srv__HTTP_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2web_interfaces__srv__HTTP_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2web_interfaces__srv__HTTP_Request__fini(&data[i - 1]);
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
ros2web_interfaces__srv__HTTP_Request__Sequence__fini(ros2web_interfaces__srv__HTTP_Request__Sequence * array)
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
      ros2web_interfaces__srv__HTTP_Request__fini(&array->data[i]);
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

ros2web_interfaces__srv__HTTP_Request__Sequence *
ros2web_interfaces__srv__HTTP_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__srv__HTTP_Request__Sequence * array = (ros2web_interfaces__srv__HTTP_Request__Sequence *)allocator.allocate(sizeof(ros2web_interfaces__srv__HTTP_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2web_interfaces__srv__HTTP_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2web_interfaces__srv__HTTP_Request__Sequence__destroy(ros2web_interfaces__srv__HTTP_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2web_interfaces__srv__HTTP_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2web_interfaces__srv__HTTP_Request__Sequence__are_equal(const ros2web_interfaces__srv__HTTP_Request__Sequence * lhs, const ros2web_interfaces__srv__HTTP_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2web_interfaces__srv__HTTP_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2web_interfaces__srv__HTTP_Request__Sequence__copy(
  const ros2web_interfaces__srv__HTTP_Request__Sequence * input,
  ros2web_interfaces__srv__HTTP_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2web_interfaces__srv__HTTP_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2web_interfaces__srv__HTTP_Request * data =
      (ros2web_interfaces__srv__HTTP_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2web_interfaces__srv__HTTP_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2web_interfaces__srv__HTTP_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2web_interfaces__srv__HTTP_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `reason`
// Member `text`
// Member `content_type`
// Member `charset`
// Member `file_path`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `body`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2web_interfaces__srv__HTTP_Response__init(ros2web_interfaces__srv__HTTP_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
    return false;
  }
  // body
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->body, 0)) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__init(&msg->content_type)) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
    return false;
  }
  // charset
  if (!rosidl_runtime_c__String__init(&msg->charset)) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__init(&msg->file_path)) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
    return false;
  }
  return true;
}

void
ros2web_interfaces__srv__HTTP_Response__fini(ros2web_interfaces__srv__HTTP_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // body
  rosidl_runtime_c__uint8__Sequence__fini(&msg->body);
  // text
  rosidl_runtime_c__String__fini(&msg->text);
  // content_type
  rosidl_runtime_c__String__fini(&msg->content_type);
  // charset
  rosidl_runtime_c__String__fini(&msg->charset);
  // file_path
  rosidl_runtime_c__String__fini(&msg->file_path);
}

bool
ros2web_interfaces__srv__HTTP_Response__are_equal(const ros2web_interfaces__srv__HTTP_Response * lhs, const ros2web_interfaces__srv__HTTP_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // body
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->body), &(rhs->body)))
  {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->content_type), &(rhs->content_type)))
  {
    return false;
  }
  // charset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->charset), &(rhs->charset)))
  {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->file_path), &(rhs->file_path)))
  {
    return false;
  }
  return true;
}

bool
ros2web_interfaces__srv__HTTP_Response__copy(
  const ros2web_interfaces__srv__HTTP_Response * input,
  ros2web_interfaces__srv__HTTP_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // body
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->body), &(output->body)))
  {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  // content_type
  if (!rosidl_runtime_c__String__copy(
      &(input->content_type), &(output->content_type)))
  {
    return false;
  }
  // charset
  if (!rosidl_runtime_c__String__copy(
      &(input->charset), &(output->charset)))
  {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__copy(
      &(input->file_path), &(output->file_path)))
  {
    return false;
  }
  return true;
}

ros2web_interfaces__srv__HTTP_Response *
ros2web_interfaces__srv__HTTP_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__srv__HTTP_Response * msg = (ros2web_interfaces__srv__HTTP_Response *)allocator.allocate(sizeof(ros2web_interfaces__srv__HTTP_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2web_interfaces__srv__HTTP_Response));
  bool success = ros2web_interfaces__srv__HTTP_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2web_interfaces__srv__HTTP_Response__destroy(ros2web_interfaces__srv__HTTP_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2web_interfaces__srv__HTTP_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2web_interfaces__srv__HTTP_Response__Sequence__init(ros2web_interfaces__srv__HTTP_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__srv__HTTP_Response * data = NULL;

  if (size) {
    data = (ros2web_interfaces__srv__HTTP_Response *)allocator.zero_allocate(size, sizeof(ros2web_interfaces__srv__HTTP_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2web_interfaces__srv__HTTP_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2web_interfaces__srv__HTTP_Response__fini(&data[i - 1]);
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
ros2web_interfaces__srv__HTTP_Response__Sequence__fini(ros2web_interfaces__srv__HTTP_Response__Sequence * array)
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
      ros2web_interfaces__srv__HTTP_Response__fini(&array->data[i]);
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

ros2web_interfaces__srv__HTTP_Response__Sequence *
ros2web_interfaces__srv__HTTP_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2web_interfaces__srv__HTTP_Response__Sequence * array = (ros2web_interfaces__srv__HTTP_Response__Sequence *)allocator.allocate(sizeof(ros2web_interfaces__srv__HTTP_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2web_interfaces__srv__HTTP_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2web_interfaces__srv__HTTP_Response__Sequence__destroy(ros2web_interfaces__srv__HTTP_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2web_interfaces__srv__HTTP_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2web_interfaces__srv__HTTP_Response__Sequence__are_equal(const ros2web_interfaces__srv__HTTP_Response__Sequence * lhs, const ros2web_interfaces__srv__HTTP_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2web_interfaces__srv__HTTP_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2web_interfaces__srv__HTTP_Response__Sequence__copy(
  const ros2web_interfaces__srv__HTTP_Response__Sequence * input,
  ros2web_interfaces__srv__HTTP_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2web_interfaces__srv__HTTP_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2web_interfaces__srv__HTTP_Response * data =
      (ros2web_interfaces__srv__HTTP_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2web_interfaces__srv__HTTP_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2web_interfaces__srv__HTTP_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2web_interfaces__srv__HTTP_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
