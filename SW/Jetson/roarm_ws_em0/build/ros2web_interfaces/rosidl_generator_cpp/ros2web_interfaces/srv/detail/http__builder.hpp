// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__BUILDER_HPP_
#define ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2web_interfaces/srv/detail/http__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2web_interfaces
{

namespace srv
{

namespace builder
{

class Init_HTTP_Request_multipart
{
public:
  explicit Init_HTTP_Request_multipart(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::srv::HTTP_Request multipart(::ros2web_interfaces::srv::HTTP_Request::_multipart_type arg)
  {
    msg_.multipart = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_charset
{
public:
  explicit Init_HTTP_Request_charset(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_multipart charset(::ros2web_interfaces::srv::HTTP_Request::_charset_type arg)
  {
    msg_.charset = std::move(arg);
    return Init_HTTP_Request_multipart(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_content_type
{
public:
  explicit Init_HTTP_Request_content_type(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_charset content_type(::ros2web_interfaces::srv::HTTP_Request::_content_type_type arg)
  {
    msg_.content_type = std::move(arg);
    return Init_HTTP_Request_charset(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_text
{
public:
  explicit Init_HTTP_Request_text(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_content_type text(::ros2web_interfaces::srv::HTTP_Request::_text_type arg)
  {
    msg_.text = std::move(arg);
    return Init_HTTP_Request_content_type(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_body
{
public:
  explicit Init_HTTP_Request_body(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_text body(::ros2web_interfaces::srv::HTTP_Request::_body_type arg)
  {
    msg_.body = std::move(arg);
    return Init_HTTP_Request_text(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_route
{
public:
  explicit Init_HTTP_Request_route(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_body route(::ros2web_interfaces::srv::HTTP_Request::_route_type arg)
  {
    msg_.route = std::move(arg);
    return Init_HTTP_Request_body(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_srv_name
{
public:
  explicit Init_HTTP_Request_srv_name(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_route srv_name(::ros2web_interfaces::srv::HTTP_Request::_srv_name_type arg)
  {
    msg_.srv_name = std::move(arg);
    return Init_HTTP_Request_route(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_query
{
public:
  explicit Init_HTTP_Request_query(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_srv_name query(::ros2web_interfaces::srv::HTTP_Request::_query_type arg)
  {
    msg_.query = std::move(arg);
    return Init_HTTP_Request_srv_name(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_path
{
public:
  explicit Init_HTTP_Request_path(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_query path(::ros2web_interfaces::srv::HTTP_Request::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_HTTP_Request_query(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_method
{
public:
  explicit Init_HTTP_Request_method(::ros2web_interfaces::srv::HTTP_Request & msg)
  : msg_(msg)
  {}
  Init_HTTP_Request_path method(::ros2web_interfaces::srv::HTTP_Request::_method_type arg)
  {
    msg_.method = std::move(arg);
    return Init_HTTP_Request_path(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

class Init_HTTP_Request_headers
{
public:
  Init_HTTP_Request_headers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HTTP_Request_method headers(::ros2web_interfaces::srv::HTTP_Request::_headers_type arg)
  {
    msg_.headers = std::move(arg);
    return Init_HTTP_Request_method(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::srv::HTTP_Request>()
{
  return ros2web_interfaces::srv::builder::Init_HTTP_Request_headers();
}

}  // namespace ros2web_interfaces


namespace ros2web_interfaces
{

namespace srv
{

namespace builder
{

class Init_HTTP_Response_file_path
{
public:
  explicit Init_HTTP_Response_file_path(::ros2web_interfaces::srv::HTTP_Response & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::srv::HTTP_Response file_path(::ros2web_interfaces::srv::HTTP_Response::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

class Init_HTTP_Response_charset
{
public:
  explicit Init_HTTP_Response_charset(::ros2web_interfaces::srv::HTTP_Response & msg)
  : msg_(msg)
  {}
  Init_HTTP_Response_file_path charset(::ros2web_interfaces::srv::HTTP_Response::_charset_type arg)
  {
    msg_.charset = std::move(arg);
    return Init_HTTP_Response_file_path(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

class Init_HTTP_Response_content_type
{
public:
  explicit Init_HTTP_Response_content_type(::ros2web_interfaces::srv::HTTP_Response & msg)
  : msg_(msg)
  {}
  Init_HTTP_Response_charset content_type(::ros2web_interfaces::srv::HTTP_Response::_content_type_type arg)
  {
    msg_.content_type = std::move(arg);
    return Init_HTTP_Response_charset(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

class Init_HTTP_Response_text
{
public:
  explicit Init_HTTP_Response_text(::ros2web_interfaces::srv::HTTP_Response & msg)
  : msg_(msg)
  {}
  Init_HTTP_Response_content_type text(::ros2web_interfaces::srv::HTTP_Response::_text_type arg)
  {
    msg_.text = std::move(arg);
    return Init_HTTP_Response_content_type(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

class Init_HTTP_Response_body
{
public:
  explicit Init_HTTP_Response_body(::ros2web_interfaces::srv::HTTP_Response & msg)
  : msg_(msg)
  {}
  Init_HTTP_Response_text body(::ros2web_interfaces::srv::HTTP_Response::_body_type arg)
  {
    msg_.body = std::move(arg);
    return Init_HTTP_Response_text(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

class Init_HTTP_Response_reason
{
public:
  explicit Init_HTTP_Response_reason(::ros2web_interfaces::srv::HTTP_Response & msg)
  : msg_(msg)
  {}
  Init_HTTP_Response_body reason(::ros2web_interfaces::srv::HTTP_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_HTTP_Response_body(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

class Init_HTTP_Response_status
{
public:
  Init_HTTP_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HTTP_Response_reason status(::ros2web_interfaces::srv::HTTP_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_HTTP_Response_reason(msg_);
  }

private:
  ::ros2web_interfaces::srv::HTTP_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::srv::HTTP_Response>()
{
  return ros2web_interfaces::srv::builder::Init_HTTP_Response_status();
}

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__BUILDER_HPP_
