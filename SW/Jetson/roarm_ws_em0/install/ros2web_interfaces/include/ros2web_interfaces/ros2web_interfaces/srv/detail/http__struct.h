// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__STRUCT_H_
#define ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'headers'
// Member 'method'
// Member 'path'
// Member 'query'
// Member 'srv_name'
// Member 'route'
// Member 'text'
// Member 'content_type'
// Member 'charset'
#include "rosidl_runtime_c/string.h"
// Member 'body'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'multipart'
#include "ros2web_interfaces/msg/detail/body_part__struct.h"

/// Struct defined in srv/HTTP in the package ros2web_interfaces.
typedef struct ros2web_interfaces__srv__HTTP_Request
{
  /// [<key>, <value>, <key>, <value>, ...]
  rosidl_runtime_c__String__Sequence headers;
  /// HTTP method,
  rosidl_runtime_c__String method;
  /// The URL including PATH INFO without the host or scheme.
  rosidl_runtime_c__String path;
  /// The query string in the URL
  rosidl_runtime_c__String query;
  rosidl_runtime_c__String srv_name;
  rosidl_runtime_c__String route;
  rosidl_runtime_c__uint8__Sequence body;
  rosidl_runtime_c__String text;
  /// ContentType
  rosidl_runtime_c__String content_type;
  rosidl_runtime_c__String charset;
  ros2web_interfaces__msg__BodyPart__Sequence multipart;
} ros2web_interfaces__srv__HTTP_Request;

// Struct for a sequence of ros2web_interfaces__srv__HTTP_Request.
typedef struct ros2web_interfaces__srv__HTTP_Request__Sequence
{
  ros2web_interfaces__srv__HTTP_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__srv__HTTP_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
// Member 'text'
// Member 'content_type'
// Member 'charset'
// Member 'file_path'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'body'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/HTTP in the package ros2web_interfaces.
typedef struct ros2web_interfaces__srv__HTTP_Response
{
  /// HTTPStatusCode
  uint16_t status;
  rosidl_runtime_c__String reason;
  rosidl_runtime_c__uint8__Sequence body;
  rosidl_runtime_c__String text;
  /// ContentType
  rosidl_runtime_c__String content_type;
  rosidl_runtime_c__String charset;
  /// int16 zlib_executor_size
  /// int16 zlib_executor
  rosidl_runtime_c__String file_path;
} ros2web_interfaces__srv__HTTP_Response;

// Struct for a sequence of ros2web_interfaces__srv__HTTP_Response.
typedef struct ros2web_interfaces__srv__HTTP_Response__Sequence
{
  ros2web_interfaces__srv__HTTP_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__srv__HTTP_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__STRUCT_H_
