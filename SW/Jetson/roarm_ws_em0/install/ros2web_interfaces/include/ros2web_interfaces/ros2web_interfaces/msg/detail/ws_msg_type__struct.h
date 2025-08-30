// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:msg/WSMsgType.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__STRUCT_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CONTINUATION'.
enum
{
  ros2web_interfaces__msg__WSMsgType__CONTINUATION = 0
};

/// Constant 'TEXT'.
enum
{
  ros2web_interfaces__msg__WSMsgType__TEXT = 1
};

/// Constant 'BINARY'.
enum
{
  ros2web_interfaces__msg__WSMsgType__BINARY = 2
};

/// Constant 'PING'.
enum
{
  ros2web_interfaces__msg__WSMsgType__PING = 11
};

/// Constant 'PONG'.
enum
{
  ros2web_interfaces__msg__WSMsgType__PONG = 12
};

/// Constant 'CLOSE'.
enum
{
  ros2web_interfaces__msg__WSMsgType__CLOSE = 21
};

/// Constant 'CLOSING'.
enum
{
  ros2web_interfaces__msg__WSMsgType__CLOSING = 22
};

/// Constant 'CLOSED'.
enum
{
  ros2web_interfaces__msg__WSMsgType__CLOSED = 23
};

/// Constant 'ERROR'.
enum
{
  ros2web_interfaces__msg__WSMsgType__ERROR = 31
};

/// Struct defined in msg/WSMsgType in the package ros2web_interfaces.
typedef struct ros2web_interfaces__msg__WSMsgType
{
  uint8_t structure_needs_at_least_one_member;
} ros2web_interfaces__msg__WSMsgType;

// Struct for a sequence of ros2web_interfaces__msg__WSMsgType.
typedef struct ros2web_interfaces__msg__WSMsgType__Sequence
{
  ros2web_interfaces__msg__WSMsgType * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__msg__WSMsgType__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__STRUCT_H_
