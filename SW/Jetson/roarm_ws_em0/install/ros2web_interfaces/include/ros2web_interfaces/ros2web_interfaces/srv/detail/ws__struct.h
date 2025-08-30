// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:srv/WS.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__WS__STRUCT_H_
#define ROS2WEB_INTERFACES__SRV__DETAIL__WS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'srv_name'
// Member 'route'
// Member 'ws_id'
// Member 'extra'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "ros2web_interfaces/msg/detail/ws_msg_data__struct.h"

/// Struct defined in srv/WS in the package ros2web_interfaces.
typedef struct ros2web_interfaces__srv__WS_Request
{
  rosidl_runtime_c__String srv_name;
  rosidl_runtime_c__String route;
  /// WSMsgType
  uint8_t type;
  rosidl_runtime_c__String ws_id;
  ros2web_interfaces__msg__WSMsgData data;
  rosidl_runtime_c__String extra;
} ros2web_interfaces__srv__WS_Request;

// Struct for a sequence of ros2web_interfaces__srv__WS_Request.
typedef struct ros2web_interfaces__srv__WS_Request__Sequence
{
  ros2web_interfaces__srv__WS_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__srv__WS_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'ws_id'
// Member 'extra'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'data'
// already included above
// #include "ros2web_interfaces/msg/detail/ws_msg_data__struct.h"

/// Struct defined in srv/WS in the package ros2web_interfaces.
typedef struct ros2web_interfaces__srv__WS_Response
{
  /// WSMsgType
  uint8_t type;
  rosidl_runtime_c__String ws_id;
  ros2web_interfaces__msg__WSMsgData data;
  rosidl_runtime_c__String extra;
} ros2web_interfaces__srv__WS_Response;

// Struct for a sequence of ros2web_interfaces__srv__WS_Response.
typedef struct ros2web_interfaces__srv__WS_Response__Sequence
{
  ros2web_interfaces__srv__WS_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__srv__WS_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__WS__STRUCT_H_
