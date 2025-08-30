// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__BUILDER_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2web_interfaces/msg/detail/ws_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2web_interfaces
{

namespace msg
{

namespace builder
{

class Init_WSMsg_extra
{
public:
  explicit Init_WSMsg_extra(::ros2web_interfaces::msg::WSMsg & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::msg::WSMsg extra(::ros2web_interfaces::msg::WSMsg::_extra_type arg)
  {
    msg_.extra = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsg msg_;
};

class Init_WSMsg_data
{
public:
  explicit Init_WSMsg_data(::ros2web_interfaces::msg::WSMsg & msg)
  : msg_(msg)
  {}
  Init_WSMsg_extra data(::ros2web_interfaces::msg::WSMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_WSMsg_extra(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsg msg_;
};

class Init_WSMsg_type
{
public:
  explicit Init_WSMsg_type(::ros2web_interfaces::msg::WSMsg & msg)
  : msg_(msg)
  {}
  Init_WSMsg_data type(::ros2web_interfaces::msg::WSMsg::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_WSMsg_data(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsg msg_;
};

class Init_WSMsg_ws_id
{
public:
  explicit Init_WSMsg_ws_id(::ros2web_interfaces::msg::WSMsg & msg)
  : msg_(msg)
  {}
  Init_WSMsg_type ws_id(::ros2web_interfaces::msg::WSMsg::_ws_id_type arg)
  {
    msg_.ws_id = std::move(arg);
    return Init_WSMsg_type(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsg msg_;
};

class Init_WSMsg_route
{
public:
  Init_WSMsg_route()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WSMsg_ws_id route(::ros2web_interfaces::msg::WSMsg::_route_type arg)
  {
    msg_.route = std::move(arg);
    return Init_WSMsg_ws_id(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::msg::WSMsg>()
{
  return ros2web_interfaces::msg::builder::Init_WSMsg_route();
}

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__BUILDER_HPP_
