// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2web_interfaces:srv/WS.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__WS__BUILDER_HPP_
#define ROS2WEB_INTERFACES__SRV__DETAIL__WS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2web_interfaces/srv/detail/ws__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2web_interfaces
{

namespace srv
{

namespace builder
{

class Init_WS_Request_extra
{
public:
  explicit Init_WS_Request_extra(::ros2web_interfaces::srv::WS_Request & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::srv::WS_Request extra(::ros2web_interfaces::srv::WS_Request::_extra_type arg)
  {
    msg_.extra = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Request msg_;
};

class Init_WS_Request_data
{
public:
  explicit Init_WS_Request_data(::ros2web_interfaces::srv::WS_Request & msg)
  : msg_(msg)
  {}
  Init_WS_Request_extra data(::ros2web_interfaces::srv::WS_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_WS_Request_extra(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Request msg_;
};

class Init_WS_Request_ws_id
{
public:
  explicit Init_WS_Request_ws_id(::ros2web_interfaces::srv::WS_Request & msg)
  : msg_(msg)
  {}
  Init_WS_Request_data ws_id(::ros2web_interfaces::srv::WS_Request::_ws_id_type arg)
  {
    msg_.ws_id = std::move(arg);
    return Init_WS_Request_data(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Request msg_;
};

class Init_WS_Request_type
{
public:
  explicit Init_WS_Request_type(::ros2web_interfaces::srv::WS_Request & msg)
  : msg_(msg)
  {}
  Init_WS_Request_ws_id type(::ros2web_interfaces::srv::WS_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_WS_Request_ws_id(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Request msg_;
};

class Init_WS_Request_route
{
public:
  explicit Init_WS_Request_route(::ros2web_interfaces::srv::WS_Request & msg)
  : msg_(msg)
  {}
  Init_WS_Request_type route(::ros2web_interfaces::srv::WS_Request::_route_type arg)
  {
    msg_.route = std::move(arg);
    return Init_WS_Request_type(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Request msg_;
};

class Init_WS_Request_srv_name
{
public:
  Init_WS_Request_srv_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WS_Request_route srv_name(::ros2web_interfaces::srv::WS_Request::_srv_name_type arg)
  {
    msg_.srv_name = std::move(arg);
    return Init_WS_Request_route(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::srv::WS_Request>()
{
  return ros2web_interfaces::srv::builder::Init_WS_Request_srv_name();
}

}  // namespace ros2web_interfaces


namespace ros2web_interfaces
{

namespace srv
{

namespace builder
{

class Init_WS_Response_extra
{
public:
  explicit Init_WS_Response_extra(::ros2web_interfaces::srv::WS_Response & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::srv::WS_Response extra(::ros2web_interfaces::srv::WS_Response::_extra_type arg)
  {
    msg_.extra = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Response msg_;
};

class Init_WS_Response_data
{
public:
  explicit Init_WS_Response_data(::ros2web_interfaces::srv::WS_Response & msg)
  : msg_(msg)
  {}
  Init_WS_Response_extra data(::ros2web_interfaces::srv::WS_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_WS_Response_extra(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Response msg_;
};

class Init_WS_Response_ws_id
{
public:
  explicit Init_WS_Response_ws_id(::ros2web_interfaces::srv::WS_Response & msg)
  : msg_(msg)
  {}
  Init_WS_Response_data ws_id(::ros2web_interfaces::srv::WS_Response::_ws_id_type arg)
  {
    msg_.ws_id = std::move(arg);
    return Init_WS_Response_data(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Response msg_;
};

class Init_WS_Response_type
{
public:
  Init_WS_Response_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WS_Response_ws_id type(::ros2web_interfaces::srv::WS_Response::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_WS_Response_ws_id(msg_);
  }

private:
  ::ros2web_interfaces::srv::WS_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::srv::WS_Response>()
{
  return ros2web_interfaces::srv::builder::Init_WS_Response_type();
}

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__WS__BUILDER_HPP_
