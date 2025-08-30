// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:msg/HTTPStatusCode.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__HTTP_STATUS_CODE__STRUCT_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__HTTP_STATUS_CODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'HTTP_OK'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_OK = 200
};

/// Constant 'HTTP_ACCEPTED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_ACCEPTED = 202
};

/// Constant 'HTTP_NON_AUTHORITATIVE_INFORMATION'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NON_AUTHORITATIVE_INFORMATION = 203
};

/// Constant 'HTTP_NO_CONTENT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NO_CONTENT = 204
};

/// Constant 'HTTP_RESET_CONTENT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_RESET_CONTENT = 205
};

/// Constant 'HTTP_PARTIAL_CONTENT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_PARTIAL_CONTENT = 206
};

/// Constant 'HTTP_MULTIPLE_CHOICES'.
/**
  * HTTP REDIRECTION
 */
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_MULTIPLE_CHOICES = 300
};

/// Constant 'HTTP_MOVED_PERMANENTLY'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_MOVED_PERMANENTLY = 301
};

/// Constant 'HTTP_FOUND'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_FOUND = 302
};

/// Constant 'HTTP_SEE_OTHER'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_SEE_OTHER = 303
};

/// Constant 'HTTP_NOT_MODIFIED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NOT_MODIFIED = 304
};

/// Constant 'HTTP_USE_PROXY'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_USE_PROXY = 305
};

/// Constant 'HTTP_TEMPORARY_REDIRECT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_TEMPORARY_REDIRECT = 307
};

/// Constant 'HTTP_PERMANENT_REDIRECT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_PERMANENT_REDIRECT = 308
};

/// Constant 'HTTP_BAD_REQUEST'.
/**
  * HTTP ERROR
  * HTTP CLIENT ERROR
 */
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_BAD_REQUEST = 400
};

/// Constant 'HTTP_UNAUTHORIZED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_UNAUTHORIZED = 401
};

/// Constant 'HTTP_PAYMENT_REQUIRED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_PAYMENT_REQUIRED = 402
};

/// Constant 'HTTP_FORBIDDEN'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_FORBIDDEN = 403
};

/// Constant 'HTTP_NOT_FOUND'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NOT_FOUND = 404
};

/// Constant 'HTTP_METHOD_NOT_ALLOWED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_METHOD_NOT_ALLOWED = 405
};

/// Constant 'HTTP_NOT_ACCEPTABLE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NOT_ACCEPTABLE = 406
};

/// Constant 'HTTP_PROXY_AUTHENTICATION_REQUIRED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_PROXY_AUTHENTICATION_REQUIRED = 407
};

/// Constant 'HTTP_REQUEST_TIMEOUT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_REQUEST_TIMEOUT = 408
};

/// Constant 'HTTP_CONFLICT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_CONFLICT = 409
};

/// Constant 'HTTP_GONE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_GONE = 410
};

/// Constant 'HTTP_LENGTH_REQUIRED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_LENGTH_REQUIRED = 411
};

/// Constant 'HTTP_PRECONDITION_FAILED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_PRECONDITION_FAILED = 412
};

/// Constant 'HTTP_REQUEST_ENTITY_TOO_LARGE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_REQUEST_ENTITY_TOO_LARGE = 413
};

/// Constant 'HTTP_REQUEST_URI_TOOLONG'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_REQUEST_URI_TOOLONG = 414
};

/// Constant 'HTTP_UNSUPPORTED_MEDIA_TYPE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_UNSUPPORTED_MEDIA_TYPE = 415
};

/// Constant 'HTTP_REQUEST_RANGE_NOT_SATISFIABLE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_REQUEST_RANGE_NOT_SATISFIABLE = 416
};

/// Constant 'HTTP_EXPECTATION_FAILED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_EXPECTATION_FAILED = 417
};

/// Constant 'HTTP_MISDIRECTED_REQUEST'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_MISDIRECTED_REQUEST = 421
};

/// Constant 'HTTP_UNPROCESSABLE_ENTITY'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_UNPROCESSABLE_ENTITY = 422
};

/// Constant 'HTTP_FAILED_DEPENDENCY'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_FAILED_DEPENDENCY = 424
};

/// Constant 'HTTP_UPGRADE_REQUIRED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_UPGRADE_REQUIRED = 426
};

/// Constant 'HTTP_PRECONDITION_REQUIRED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_PRECONDITION_REQUIRED = 428
};

/// Constant 'HTTP_TOO_MANY_REQUESTS'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_TOO_MANY_REQUESTS = 429
};

/// Constant 'HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE = 431
};

/// Constant 'HTTP_UNAVAILABLE_FOR_LEGAL_REASONS'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_UNAVAILABLE_FOR_LEGAL_REASONS = 451
};

/// Constant 'HTTP_INTERNAL_SERVER_ERROR'.
/**
  * HTTP SERVER ERROR
 */
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_INTERNAL_SERVER_ERROR = 500
};

/// Constant 'HTTP_NOT_IMPLEMENTED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NOT_IMPLEMENTED = 501
};

/// Constant 'HTTP_BAD_GATEWAY'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_BAD_GATEWAY = 502
};

/// Constant 'HTTP_SERVICE_UNAVAILABLE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_SERVICE_UNAVAILABLE = 503
};

/// Constant 'HTTP_GATEWAY_TIMEOUT'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_GATEWAY_TIMEOUT = 504
};

/// Constant 'HTTP_VERSION_NOT_SUPPORTED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_VERSION_NOT_SUPPORTED = 505
};

/// Constant 'HTTP_VARIANT_ALSO_NEGOTIATES'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_VARIANT_ALSO_NEGOTIATES = 506
};

/// Constant 'HTTP_INSUFFICIENT_STORAGE'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_INSUFFICIENT_STORAGE = 507
};

/// Constant 'HTTP_NOT_EXTENDED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NOT_EXTENDED = 510
};

/// Constant 'HTTP_NETWORK_AUTHENTICATION_REQUIRED'.
enum
{
  ros2web_interfaces__msg__HTTPStatusCode__HTTP_NETWORK_AUTHENTICATION_REQUIRED = 511
};

/// Struct defined in msg/HTTPStatusCode in the package ros2web_interfaces.
/**
  * HTTP Successful
 */
typedef struct ros2web_interfaces__msg__HTTPStatusCode
{
  uint8_t structure_needs_at_least_one_member;
} ros2web_interfaces__msg__HTTPStatusCode;

// Struct for a sequence of ros2web_interfaces__msg__HTTPStatusCode.
typedef struct ros2web_interfaces__msg__HTTPStatusCode__Sequence
{
  ros2web_interfaces__msg__HTTPStatusCode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__msg__HTTPStatusCode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__HTTP_STATUS_CODE__STRUCT_H_
