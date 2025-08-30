// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__TRAITS_HPP_
#define ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2web_interfaces/srv/detail/http__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'multipart'
#include "ros2web_interfaces/msg/detail/body_part__traits.hpp"

namespace ros2web_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const HTTP_Request & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: method
  {
    out << "method: ";
    rosidl_generator_traits::value_to_yaml(msg.method, out);
    out << ", ";
  }

  // member: path
  {
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
    out << ", ";
  }

  // member: query
  {
    out << "query: ";
    rosidl_generator_traits::value_to_yaml(msg.query, out);
    out << ", ";
  }

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

  // member: body
  {
    if (msg.body.size() == 0) {
      out << "body: []";
    } else {
      out << "body: [";
      size_t pending_items = msg.body.size();
      for (auto item : msg.body) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: text
  {
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << ", ";
  }

  // member: content_type
  {
    out << "content_type: ";
    rosidl_generator_traits::value_to_yaml(msg.content_type, out);
    out << ", ";
  }

  // member: charset
  {
    out << "charset: ";
    rosidl_generator_traits::value_to_yaml(msg.charset, out);
    out << ", ";
  }

  // member: multipart
  {
    if (msg.multipart.size() == 0) {
      out << "multipart: []";
    } else {
      out << "multipart: [";
      size_t pending_items = msg.multipart.size();
      for (auto item : msg.multipart) {
        to_flow_style_yaml(item, out);
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
  const HTTP_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: method
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "method: ";
    rosidl_generator_traits::value_to_yaml(msg.method, out);
    out << "\n";
  }

  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
    out << "\n";
  }

  // member: query
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "query: ";
    rosidl_generator_traits::value_to_yaml(msg.query, out);
    out << "\n";
  }

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

  // member: body
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.body.size() == 0) {
      out << "body: []\n";
    } else {
      out << "body:\n";
      for (auto item : msg.body) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
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

  // member: charset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charset: ";
    rosidl_generator_traits::value_to_yaml(msg.charset, out);
    out << "\n";
  }

  // member: multipart
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.multipart.size() == 0) {
      out << "multipart: []\n";
    } else {
      out << "multipart:\n";
      for (auto item : msg.multipart) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HTTP_Request & msg, bool use_flow_style = false)
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
  const ros2web_interfaces::srv::HTTP_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::srv::HTTP_Request & msg)
{
  return ros2web_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::srv::HTTP_Request>()
{
  return "ros2web_interfaces::srv::HTTP_Request";
}

template<>
inline const char * name<ros2web_interfaces::srv::HTTP_Request>()
{
  return "ros2web_interfaces/srv/HTTP_Request";
}

template<>
struct has_fixed_size<ros2web_interfaces::srv::HTTP_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::srv::HTTP_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::srv::HTTP_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2web_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const HTTP_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: body
  {
    if (msg.body.size() == 0) {
      out << "body: []";
    } else {
      out << "body: [";
      size_t pending_items = msg.body.size();
      for (auto item : msg.body) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: text
  {
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << ", ";
  }

  // member: content_type
  {
    out << "content_type: ";
    rosidl_generator_traits::value_to_yaml(msg.content_type, out);
    out << ", ";
  }

  // member: charset
  {
    out << "charset: ";
    rosidl_generator_traits::value_to_yaml(msg.charset, out);
    out << ", ";
  }

  // member: file_path
  {
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HTTP_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: body
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.body.size() == 0) {
      out << "body: []\n";
    } else {
      out << "body:\n";
      for (auto item : msg.body) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
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

  // member: charset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charset: ";
    rosidl_generator_traits::value_to_yaml(msg.charset, out);
    out << "\n";
  }

  // member: file_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HTTP_Response & msg, bool use_flow_style = false)
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
  const ros2web_interfaces::srv::HTTP_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2web_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2web_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2web_interfaces::srv::HTTP_Response & msg)
{
  return ros2web_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2web_interfaces::srv::HTTP_Response>()
{
  return "ros2web_interfaces::srv::HTTP_Response";
}

template<>
inline const char * name<ros2web_interfaces::srv::HTTP_Response>()
{
  return "ros2web_interfaces/srv/HTTP_Response";
}

template<>
struct has_fixed_size<ros2web_interfaces::srv::HTTP_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2web_interfaces::srv::HTTP_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2web_interfaces::srv::HTTP_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2web_interfaces::srv::HTTP>()
{
  return "ros2web_interfaces::srv::HTTP";
}

template<>
inline const char * name<ros2web_interfaces::srv::HTTP>()
{
  return "ros2web_interfaces/srv/HTTP";
}

template<>
struct has_fixed_size<ros2web_interfaces::srv::HTTP>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2web_interfaces::srv::HTTP_Request>::value &&
    has_fixed_size<ros2web_interfaces::srv::HTTP_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2web_interfaces::srv::HTTP>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2web_interfaces::srv::HTTP_Request>::value &&
    has_bounded_size<ros2web_interfaces::srv::HTTP_Response>::value
  >
{
};

template<>
struct is_service<ros2web_interfaces::srv::HTTP>
  : std::true_type
{
};

template<>
struct is_service_request<ros2web_interfaces::srv::HTTP_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2web_interfaces::srv::HTTP_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__TRAITS_HPP_
