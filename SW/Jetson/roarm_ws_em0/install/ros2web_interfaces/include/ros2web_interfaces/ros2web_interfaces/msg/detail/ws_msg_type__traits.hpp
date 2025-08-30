// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2web_interfaces:msg/WSMsgType.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__TRAITS_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2web_interfaces/msg/detail/ws_msg_type__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2web_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WSMsgType & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WSMsgType & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WSMsgType & msg, bool use_flow_style = false)
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
  const ros2web_interfaces::msg::WSMsgType & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::msg::WSMsgType & msg)
{
  return ros2web_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::msg::WSMsgType>()
{
  return "ros2web_interfaces::msg::WSMsgType";
}

template<>
inline const char * name<ros2web_interfaces::msg::WSMsgType>()
{
  return "ros2web_interfaces/msg/WSMsgType";
}

template<>
struct has_fixed_size<ros2web_interfaces::msg::WSMsgType>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2web_interfaces::msg::WSMsgType>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2web_interfaces::msg::WSMsgType>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__TRAITS_HPP_
