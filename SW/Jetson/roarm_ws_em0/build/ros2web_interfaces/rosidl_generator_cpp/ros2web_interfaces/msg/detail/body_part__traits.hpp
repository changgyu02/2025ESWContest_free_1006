// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2web_interfaces:msg/BodyPart.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__TRAITS_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2web_interfaces/msg/detail/body_part__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2web_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BodyPart & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << ", ";
  }

  // member: content_type
  {
    out << "content_type: ";
    rosidl_generator_traits::value_to_yaml(msg.content_type, out);
    out << ", ";
  }

  // member: headers
  {
    if (msg.headers.size() == 0) {
      out << "headers: []";
    } else {
      out << "headers: [";
      size_t pending_items = msg.headers.size();
      for (auto item : msg.headers) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
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
  const BodyPart & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }

  // member: content_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "content_type: ";
    rosidl_generator_traits::value_to_yaml(msg.content_type, out);
    out << "\n";
  }

  // member: headers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.headers.size() == 0) {
      out << "headers: []\n";
    } else {
      out << "headers:\n";
      for (auto item : msg.headers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
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

inline std::string to_yaml(const BodyPart & msg, bool use_flow_style = false)
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
  const ros2web_interfaces::msg::BodyPart & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::msg::BodyPart & msg)
{
  return ros2web_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::msg::BodyPart>()
{
  return "ros2web_interfaces::msg::BodyPart";
}

template<>
inline const char * name<ros2web_interfaces::msg::BodyPart>()
{
  return "ros2web_interfaces/msg/BodyPart";
}

template<>
struct has_fixed_size<ros2web_interfaces::msg::BodyPart>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::msg::BodyPart>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::msg::BodyPart>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__TRAITS_HPP_
