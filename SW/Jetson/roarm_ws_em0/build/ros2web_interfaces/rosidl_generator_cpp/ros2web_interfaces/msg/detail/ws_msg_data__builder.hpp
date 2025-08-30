// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2web_interfaces:msg/WSMsgData.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__BUILDER_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2web_interfaces/msg/detail/ws_msg_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2web_interfaces
{

namespace msg
{

namespace builder
{

class Init_WSMsgData_bytes
{
public:
  explicit Init_WSMsgData_bytes(::ros2web_interfaces::msg::WSMsgData & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::msg::WSMsgData bytes(::ros2web_interfaces::msg::WSMsgData::_bytes_type arg)
  {
    msg_.bytes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsgData msg_;
};

class Init_WSMsgData_str
{
public:
  Init_WSMsgData_str()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WSMsgData_bytes str(::ros2web_interfaces::msg::WSMsgData::_str_type arg)
  {
    msg_.str = std::move(arg);
    return Init_WSMsgData_bytes(msg_);
  }

private:
  ::ros2web_interfaces::msg::WSMsgData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::msg::WSMsgData>()
{
  return ros2web_interfaces::msg::builder::Init_WSMsgData_str();
}

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__BUILDER_HPP_
