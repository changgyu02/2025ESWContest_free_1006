// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:msg/BodyPart.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__STRUCT_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'filename'
// Member 'content_type'
// Member 'headers'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BodyPart in the package ros2web_interfaces.
typedef struct ros2web_interfaces__msg__BodyPart
{
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String filename;
  rosidl_runtime_c__String content_type;
  rosidl_runtime_c__String__Sequence headers;
  rosidl_runtime_c__uint8__Sequence data;
} ros2web_interfaces__msg__BodyPart;

// Struct for a sequence of ros2web_interfaces__msg__BodyPart.
typedef struct ros2web_interfaces__msg__BodyPart__Sequence
{
  ros2web_interfaces__msg__BodyPart * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__msg__BodyPart__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__STRUCT_H_
