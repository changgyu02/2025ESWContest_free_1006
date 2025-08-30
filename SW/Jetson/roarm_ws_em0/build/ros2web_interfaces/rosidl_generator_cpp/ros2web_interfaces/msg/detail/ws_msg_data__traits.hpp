// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2web_interfaces:msg/WSMsgData.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__TRAITS_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2web_interfaces/msg/detail/ws_msg_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2web_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WSMsgData & msg,
  std::ostream & out)
{
  out << "{";
  // member: str
  {
    out << "str: ";
    rosidl_generator_traits::value_to_yaml(msg.str, out);
    out << ", ";
  }

  // member: bytes
  {
    if (msg.bytes.size() == 0) {
      out << "bytes: []";
    } else {
      out << "bytes: [";
      size_t pending_items = msg.bytes.size();
      for (auto item : msg.bytes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WSMsgData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: str
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "str: ";
    rosidl_generator_traits::value_to_yaml(msg.str, out);
    out << "\n";
  }

  // member: bytes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bytes.size() == 0) {
      out << "bytes: []\n";
    } else {
      out << "bytes:\n";
      for (auto item : msg.bytes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WSMsgData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros2web_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2web_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2web_interfaces::msg::WSMsgData & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::msg::WSMsgData & msg)
{
  return ros2web_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::msg::WSMsgData>()
{
  return "ros2web_interfaces::msg::WSMsgData";
}

template<>
inline const char * name<ros2web_interfaces::msg::WSMsgData>()
{
  return "ros2web_interfaces/msg/WSMsgData";
}

template<>
struct has_fixed_size<ros2web_interfaces::msg::WSMsgData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::msg::WSMsgData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::msg::WSMsgData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__TRAITS_HPP_
