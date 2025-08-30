// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2web_interfaces:srv/WS.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__WS__TRAITS_HPP_
#define ROS2WEB_INTERFACES__SRV__DETAIL__WS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2web_interfaces/srv/detail/ws__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'data'
#include "ros2web_interfaces/msg/detail/ws_msg_data__traits.hpp"

namespace ros2web_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const WS_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: srv_name
  {
    out << "srv_name: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_name, out);
    out << ", ";
  }

  // member: route
  {
    out << "route: ";
    rosidl_generator_traits::value_to_yaml(msg.route, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: ws_id
  {
    out << "ws_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ws_id, out);
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
  const WS_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: srv_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_name: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_name, out);
    out << "\n";
  }

  // member: route
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route: ";
    rosidl_generator_traits::value_to_yaml(msg.route, out);
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

  // member: ws_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ws_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ws_id, out);
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

inline std::string to_yaml(const WS_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2web_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2web_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2web_interfaces::srv::WS_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::srv::WS_Request & msg)
{
  return ros2web_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::srv::WS_Request>()
{
  return "ros2web_interfaces::srv::WS_Request";
}

template<>
inline const char * name<ros2web_interfaces::srv::WS_Request>()
{
  return "ros2web_interfaces/srv/WS_Request";
}

template<>
struct has_fixed_size<ros2web_interfaces::srv::WS_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::srv::WS_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::srv::WS_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'data'
// already included above
// #include "ros2web_interfaces/msg/detail/ws_msg_data__traits.hpp"

namespace ros2web_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const WS_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: ws_id
  {
    out << "ws_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ws_id, out);
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
  const WS_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
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

inline std::string to_yaml(const WS_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2web_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2web_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2web_interfaces::srv::WS_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::srv::WS_Response & msg)
{
  return ros2web_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::srv::WS_Response>()
{
  return "ros2web_interfaces::srv::WS_Response";
}

template<>
inline const char * name<ros2web_interfaces::srv::WS_Response>()
{
  return "ros2web_interfaces/srv/WS_Response";
}

template<>
struct has_fixed_size<ros2web_interfaces::srv::WS_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::srv::WS_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::srv::WS_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2web_interfaces::srv::WS>()
{
  return "ros2web_interfaces::srv::WS";
}

template<>
inline const char * name<ros2web_interfaces::srv::WS>()
{
  return "ros2web_interfaces/srv/WS";
}

template<>
struct has_fixed_size<ros2web_interfaces::srv::WS>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2web_interfaces::srv::WS_Request>::value &&
    has_fixed_size<ros2web_interfaces::srv::WS_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2web_interfaces::srv::WS>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2web_interfaces::srv::WS_Request>::value &&
    has_bounded_size<ros2web_interfaces::srv::WS_Response>::value
  >
{
};

template<>
struct is_service<ros2web_interfaces::srv::WS>
  : std::true_type
{
};

template<>
struct is_service_request<ros2web_interfaces::srv::WS_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2web_interfaces::srv::WS_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__WS__TRAITS_HPP_
