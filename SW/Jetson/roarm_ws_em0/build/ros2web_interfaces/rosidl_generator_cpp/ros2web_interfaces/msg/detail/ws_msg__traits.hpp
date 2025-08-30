// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__TRAITS_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2web_interfaces/msg/detail/ws_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'data'
#include "ros2web_interfaces/msg/detail/ws_msg_data__traits.hpp"

namespace ros2web_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WSMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: route
  {
    out << "route: ";
    rosidl_generator_traits::value_to_yaml(msg.route, out);
    out << ", ";
  }

  // member: ws_id
  {
    out << "ws_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ws_id, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    to_flow_style_yaml(msg.data, out);
    out << ", ";
  }

  // member: extra
  {
    out << "extra: ";
    rosidl_generator_traits::value_to_yaml(msg.extra, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WSMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: route
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route: ";
    rosidl_generator_traits::value_to_yaml(msg.route, out);
    out << "\n";
  }

  // member: ws_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ws_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ws_id, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data:\n";
    to_block_style_yaml(msg.data, out, indentation + 2);
  }

  // member: extra
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "extra: ";
    rosidl_generator_traits::value_to_yaml(msg.extra, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WSMsg & msg, bool use_flow_style = false)
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
  const ros2web_interfaces::msg::WSMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::msg::WSMsg & msg)
{
  return ros2web_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::msg::WSMsg>()
{
  return "ros2web_interfaces::msg::WSMsg";
}

template<>
inline const char * name<ros2web_interfaces::msg::WSMsg>()
{
  return "ros2web_interfaces/msg/WSMsg";
}

template<>
struct has_fixed_size<ros2web_interfaces::msg::WSMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::msg::WSMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::msg::WSMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__TRAITS_HPP_
