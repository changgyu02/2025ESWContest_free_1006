// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__STRUCT_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'route'
// Member 'ws_id'
// Member 'extra'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "ros2web_interfaces/msg/detail/ws_msg_data__struct.h"

/// Struct defined in msg/WSMsg in the package ros2web_interfaces.
typedef struct ros2web_interfaces__msg__WSMsg
{
  rosidl_runtime_c__String route;
  rosidl_runtime_c__String ws_id;
  /// WSMsgType
  uint8_t type;
  ros2web_interfaces__msg__WSMsgData data;
  rosidl_runtime_c__String extra;
} ros2web_interfaces__msg__WSMsg;

// Struct for a sequence of ros2web_interfaces__msg__WSMsg.
typedef struct ros2web_interfaces__msg__WSMsg__Sequence
{
  ros2web_interfaces__msg__WSMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__msg__WSMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__STRUCT_H_
