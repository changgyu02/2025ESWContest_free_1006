// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/msg/detail/ws_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2web_interfaces/msg/detail/ws_msg__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace ros2web_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros2web_interfaces::msg::WSMsgData &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2web_interfaces::msg::WSMsgData &);
size_t get_serialized_size(
  const ros2web_interfaces::msg::WSMsgData &,
  size_t current_alignment);
size_t
max_serialized_size_WSMsgData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2web_interfaces


namespace ros2web_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2web_interfaces
cdr_serialize(
  const ros2web_interfaces::msg::WSMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: route
  cdr << ros_message.route;
  // Member: ws_id
  cdr << ros_message.ws_id;
  // Member: type
  cdr << ros_message.type;
  // Member: data
  ros2web_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.data,
    cdr);
  // Member: extra
  cdr << ros_message.extra;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2web_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2web_interfaces::msg::WSMsg & ros_message)
{
  // Member: route
  cdr >> ros_message.route;

  // Member: ws_id
  cdr >> ros_message.ws_id;

  // Member: type
  cdr >> ros_message.type;

  // Member: data
  ros2web_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.data);

  // Member: extra
  cdr >> ros_message.extra;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2web_interfaces
get_serialized_size(
  const ros2web_interfaces::msg::WSMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: route
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.route.size() + 1);
  // Member: ws_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.ws_id.size() + 1);
  // Member: type
  {
    size_t item_size = sizeof(ros_message.type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: data

  current_alignment +=
    ros2web_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.data, current_alignment);
  // Member: extra
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.extra.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2web_interfaces
max_serialized_size_WSMsg(
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


  // Member: route
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

  // Member: ws_id
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

  // Member: type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2web_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_WSMsgData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: extra
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
    using DataType = ros2web_interfaces::msg::WSMsg;
    is_plain =
      (
      offsetof(DataType, extra) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _WSMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2web_interfaces::msg::WSMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _WSMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2web_interfaces::msg::WSMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _WSMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2web_interfaces::msg::WSMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _WSMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_WSMsg(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _WSMsg__callbacks = {
  "ros2web_interfaces::msg",
  "WSMsg",
  _WSMsg__cdr_serialize,
  _WSMsg__cdr_deserialize,
  _WSMsg__get_serialized_size,
  _WSMsg__max_serialized_size
};

static rosidl_message_type_support_t _WSMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_WSMsg__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros2web_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2web_interfaces::msg::WSMsg>()
{
  return &ros2web_interfaces::msg::typesupport_fastrtps_cpp::_WSMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2web_interfaces, msg, WSMsg)() {
  return &ros2web_interfaces::msg::typesupport_fastrtps_cpp::_WSMsg__handle;
}

#ifdef __cplusplus
}
#endif
