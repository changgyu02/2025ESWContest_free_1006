// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/msg/detail/ws_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2web_interfaces/msg/detail/ws_msg__struct.h"
#include "ros2web_interfaces/msg/detail/ws_msg__functions.h"
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

#include "ros2web_interfaces/msg/detail/ws_msg_data__functions.h"  // data
#include "rosidl_runtime_c/string.h"  // extra, route, ws_id
#include "rosidl_runtime_c/string_functions.h"  // extra, route, ws_id

// forward declare type support functions
size_t get_serialized_size_ros2web_interfaces__msg__WSMsgData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2web_interfaces__msg__WSMsgData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData)();


using _WSMsg__ros_msg_type = ros2web_interfaces__msg__WSMsg;

static bool _WSMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WSMsg__ros_msg_type * ros_message = static_cast<const _WSMsg__ros_msg_type *>(untyped_ros_message);
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

  // Field name: ws_id
  {
    const rosidl_runtime_c__String * str = &ros_message->ws_id;
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

  // Field name: type
  {
    cdr << ros_message->type;
  }

  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->data, cdr))
    {
      return false;
    }
  }

  // Field name: extra
  {
    const rosidl_runtime_c__String * str = &ros_message->extra;
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

static bool _WSMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WSMsg__ros_msg_type * ros_message = static_cast<_WSMsg__ros_msg_type *>(untyped_ros_message);
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

  // Field name: ws_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->ws_id.data) {
      rosidl_runtime_c__String__init(&ros_message->ws_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->ws_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'ws_id'\n");
      return false;
    }
  }

  // Field name: type
  {
    cdr >> ros_message->type;
  }

  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->data))
    {
      return false;
    }
  }

  // Field name: extra
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->extra.data) {
      rosidl_runtime_c__String__init(&ros_message->extra);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->extra,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'extra'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t get_serialized_size_ros2web_interfaces__msg__WSMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WSMsg__ros_msg_type * ros_message = static_cast<const _WSMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name route
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->route.size + 1);
  // field.name ws_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->ws_id.size + 1);
  // field.name type
  {
    size_t item_size = sizeof(ros_message->type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name data

  current_alignment += get_serialized_size_ros2web_interfaces__msg__WSMsgData(
    &(ros_message->data), current_alignment);
  // field.name extra
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->extra.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _WSMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2web_interfaces__msg__WSMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t max_serialized_size_ros2web_interfaces__msg__WSMsg(
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
  // member: ws_id
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
  // member: type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2web_interfaces__msg__WSMsgData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: extra
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
    using DataType = ros2web_interfaces__msg__WSMsg;
    is_plain =
      (
      offsetof(DataType, extra) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _WSMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2web_interfaces__msg__WSMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_WSMsg = {
  "ros2web_interfaces::msg",
  "WSMsg",
  _WSMsg__cdr_serialize,
  _WSMsg__cdr_deserialize,
  _WSMsg__get_serialized_size,
  _WSMsg__max_serialized_size
};

static rosidl_message_type_support_t _WSMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WSMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsg)() {
  return &_WSMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
