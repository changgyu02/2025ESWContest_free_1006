// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/srv/detail/http__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2web_interfaces/srv/detail/http__struct.h"
#include "ros2web_interfaces/srv/detail/http__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "ros2web_interfaces/msg/detail/body_part__functions.h"  // multipart
#include "rosidl_runtime_c/primitives_sequence.h"  // body
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // body
#include "rosidl_runtime_c/string.h"  // charset, content_type, headers, method, path, query, route, srv_name, text
#include "rosidl_runtime_c/string_functions.h"  // charset, content_type, headers, method, path, query, route, srv_name, text

// forward declare type support functions
size_t get_serialized_size_ros2web_interfaces__msg__BodyPart(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2web_interfaces__msg__BodyPart(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, BodyPart)();


using _HTTP_Request__ros_msg_type = ros2web_interfaces__srv__HTTP_Request;

static bool _HTTP_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _HTTP_Request__ros_msg_type * ros_message = static_cast<const _HTTP_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: headers
  {
    size_t size = ros_message->headers.size;
    auto array_ptr = ros_message->headers.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: method
  {
    const rosidl_runtime_c__String * str = &ros_message->method;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: path
  {
    const rosidl_runtime_c__String * str = &ros_message->path;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: query
  {
    const rosidl_runtime_c__String * str = &ros_message->query;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: srv_name
  {
    const rosidl_runtime_c__String * str = &ros_message->srv_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: route
  {
    const rosidl_runtime_c__String * str = &ros_message->route;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: body
  {
    size_t size = ros_message->body.size;
    auto array_ptr = ros_message->body.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: text
  {
    const rosidl_runtime_c__String * str = &ros_message->text;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: content_type
  {
    const rosidl_runtime_c__String * str = &ros_message->content_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: charset
  {
    const rosidl_runtime_c__String * str = &ros_message->charset;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: multipart
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, BodyPart
      )()->data);
    size_t size = ros_message->multipart.size;
    auto array_ptr = ros_message->multipart.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _HTTP_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _HTTP_Request__ros_msg_type * ros_message = static_cast<_HTTP_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: headers
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->headers.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->headers);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->headers, size)) {
      fprintf(stderr, "failed to create array for field 'headers'");
      return false;
    }
    auto array_ptr = ros_message->headers.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'headers'\n");
        return false;
      }
    }
  }

  // Field name: method
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->method.data) {
      rosidl_runtime_c__String__init(&ros_message->method);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->method,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'method'\n");
      return false;
    }
  }

  // Field name: path
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->path.data) {
      rosidl_runtime_c__String__init(&ros_message->path);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->path,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'path'\n");
      return false;
    }
  }

  // Field name: query
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->query.data) {
      rosidl_runtime_c__String__init(&ros_message->query);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->query,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'query'\n");
      return false;
    }
  }

  // Field name: srv_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->srv_name.data) {
      rosidl_runtime_c__String__init(&ros_message->srv_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->srv_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'srv_name'\n");
      return false;
    }
  }

  // Field name: route
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->route.data) {
      rosidl_runtime_c__String__init(&ros_message->route);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->route,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'route'\n");
      return false;
    }
  }

  // Field name: body
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->body.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->body);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->body, size)) {
      fprintf(stderr, "failed to create array for field 'body'");
      return false;
    }
    auto array_ptr = ros_message->body.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: text
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->text.data) {
      rosidl_runtime_c__String__init(&ros_message->text);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->text,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'text'\n");
      return false;
    }
  }

  // Field name: content_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->content_type.data) {
      rosidl_runtime_c__String__init(&ros_message->content_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->content_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'content_type'\n");
      return false;
    }
  }

  // Field name: charset
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->charset.data) {
      rosidl_runtime_c__String__init(&ros_message->charset);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->charset,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'charset'\n");
      return false;
    }
  }

  // Field name: multipart
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, BodyPart
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->multipart.data) {
      ros2web_interfaces__msg__BodyPart__Sequence__fini(&ros_message->multipart);
    }
    if (!ros2web_interfaces__msg__BodyPart__Sequence__init(&ros_message->multipart, size)) {
      fprintf(stderr, "failed to create array for field 'multipart'");
      return false;
    }
    auto array_ptr = ros_message->multipart.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t get_serialized_size_ros2web_interfaces__srv__HTTP_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _HTTP_Request__ros_msg_type * ros_message = static_cast<const _HTTP_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name headers
  {
    size_t array_size = ros_message->headers.size;
    auto array_ptr = ros_message->headers.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name method
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->method.size + 1);
  // field.name path
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->path.size + 1);
  // field.name query
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->query.size + 1);
  // field.name srv_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->srv_name.size + 1);
  // field.name route
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->route.size + 1);
  // field.name body
  {
    size_t array_size = ros_message->body.size;
    auto array_ptr = ros_message->body.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name text
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->text.size + 1);
  // field.name content_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->content_type.size + 1);
  // field.name charset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->charset.size + 1);
  // field.name multipart
  {
    size_t array_size = ros_message->multipart.size;
    auto array_ptr = ros_message->multipart.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ros2web_interfaces__msg__BodyPart(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _HTTP_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2web_interfaces__srv__HTTP_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t max_serialized_size_ros2web_interfaces__srv__HTTP_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: headers
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: method
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: path
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: query
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: srv_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: route
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: body
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: text
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: content_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: charset
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: multipart
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2web_interfaces__msg__BodyPart(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2web_interfaces__srv__HTTP_Request;
    is_plain =
      (
      offsetof(DataType, multipart) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _HTTP_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2web_interfaces__srv__HTTP_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_HTTP_Request = {
  "ros2web_interfaces::srv",
  "HTTP_Request",
  _HTTP_Request__cdr_serialize,
  _HTTP_Request__cdr_deserialize,
  _HTTP_Request__get_serialized_size,
  _HTTP_Request__max_serialized_size
};

static rosidl_message_type_support_t _HTTP_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_HTTP_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, HTTP_Request)() {
  return &_HTTP_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2web_interfaces/srv/detail/http__struct.h"
// already included above
// #include "ros2web_interfaces/srv/detail/http__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"  // body
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"  // body
// already included above
// #include "rosidl_runtime_c/string.h"  // charset, content_type, file_path, reason, text
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // charset, content_type, file_path, reason, text

// forward declare type support functions


using _HTTP_Response__ros_msg_type = ros2web_interfaces__srv__HTTP_Response;

static bool _HTTP_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _HTTP_Response__ros_msg_type * ros_message = static_cast<const _HTTP_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: status
  {
    cdr << ros_message->status;
  }

  // Field name: reason
  {
    const rosidl_runtime_c__String * str = &ros_message->reason;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: body
  {
    size_t size = ros_message->body.size;
    auto array_ptr = ros_message->body.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: text
  {
    const rosidl_runtime_c__String * str = &ros_message->text;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: content_type
  {
    const rosidl_runtime_c__String * str = &ros_message->content_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: charset
  {
    const rosidl_runtime_c__String * str = &ros_message->charset;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: file_path
  {
    const rosidl_runtime_c__String * str = &ros_message->file_path;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _HTTP_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _HTTP_Response__ros_msg_type * ros_message = static_cast<_HTTP_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: status
  {
    cdr >> ros_message->status;
  }

  // Field name: reason
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->reason.data) {
      rosidl_runtime_c__String__init(&ros_message->reason);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->reason,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'reason'\n");
      return false;
    }
  }

  // Field name: body
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->body.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->body);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->body, size)) {
      fprintf(stderr, "failed to create array for field 'body'");
      return false;
    }
    auto array_ptr = ros_message->body.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: text
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->text.data) {
      rosidl_runtime_c__String__init(&ros_message->text);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->text,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'text'\n");
      return false;
    }
  }

  // Field name: content_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->content_type.data) {
      rosidl_runtime_c__String__init(&ros_message->content_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->content_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'content_type'\n");
      return false;
    }
  }

  // Field name: charset
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->charset.data) {
      rosidl_runtime_c__String__init(&ros_message->charset);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->charset,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'charset'\n");
      return false;
    }
  }

  // Field name: file_path
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->file_path.data) {
      rosidl_runtime_c__String__init(&ros_message->file_path);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->file_path,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'file_path'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t get_serialized_size_ros2web_interfaces__srv__HTTP_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _HTTP_Response__ros_msg_type * ros_message = static_cast<const _HTTP_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name status
  {
    size_t item_size = sizeof(ros_message->status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->reason.size + 1);
  // field.name body
  {
    size_t array_size = ros_message->body.size;
    auto array_ptr = ros_message->body.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name text
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->text.size + 1);
  // field.name content_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->content_type.size + 1);
  // field.name charset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->charset.size + 1);
  // field.name file_path
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->file_path.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _HTTP_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2web_interfaces__srv__HTTP_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t max_serialized_size_ros2web_interfaces__srv__HTTP_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: reason
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: body
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: text
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: content_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: charset
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: file_path
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2web_interfaces__srv__HTTP_Response;
    is_plain =
      (
      offsetof(DataType, file_path) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _HTTP_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2web_interfaces__srv__HTTP_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_HTTP_Response = {
  "ros2web_interfaces::srv",
  "HTTP_Response",
  _HTTP_Response__cdr_serialize,
  _HTTP_Response__cdr_deserialize,
  _HTTP_Response__get_serialized_size,
  _HTTP_Response__max_serialized_size
};

static rosidl_message_type_support_t _HTTP_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_HTTP_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, HTTP_Response)() {
  return &_HTTP_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2web_interfaces/srv/http.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t HTTP__callbacks = {
  "ros2web_interfaces::srv",
  "HTTP",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, HTTP_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, HTTP_Response)(),
};

static rosidl_service_type_support_t HTTP__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &HTTP__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, HTTP)() {
  return &HTTP__handle;
}

#if defined(__cplusplus)
}
#endif
