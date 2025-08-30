// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:msg/WSMsgData.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__STRUCT_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'str'
#include "rosidl_runtime_c/string.h"
// Member 'bytes'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/WSMsgData in the package ros2web_interfaces.
typedef struct ros2web_interfaces__msg__WSMsgData
{
  rosidl_runtime_c__String str;
  rosidl_runtime_c__uint8__Sequence bytes;
} ros2web_interfaces__msg__WSMsgData;

// Struct for a sequence of ros2web_interfaces__msg__WSMsgData.
typedef struct ros2web_interfaces__msg__WSMsgData__Sequence
{
  ros2web_interfaces__msg__WSMsgData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__msg__WSMsgData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__STRUCT_H_
