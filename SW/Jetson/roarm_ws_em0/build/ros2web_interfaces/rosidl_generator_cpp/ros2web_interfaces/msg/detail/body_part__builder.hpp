// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2web_interfaces:msg/BodyPart.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__BUILDER_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2web_interfaces/msg/detail/body_part__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2web_interfaces
{

namespace msg
{

namespace builder
{

class Init_BodyPart_data
{
public:
  explicit Init_BodyPart_data(::ros2web_interfaces::msg::BodyPart & msg)
  : msg_(msg)
  {}
  ::ros2web_interfaces::msg::BodyPart data(::ros2web_interfaces::msg::BodyPart::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2web_interfaces::msg::BodyPart msg_;
};

class Init_BodyPart_headers
{
public:
  explicit Init_BodyPart_headers(::ros2web_interfaces::msg::BodyPart & msg)
  : msg_(msg)
  {}
  Init_BodyPart_data headers(::ros2web_interfaces::msg::BodyPart::_headers_type arg)
  {
    msg_.headers = std::move(arg);
    return Init_BodyPart_data(msg_);
  }

private:
  ::ros2web_interfaces::msg::BodyPart msg_;
};

class Init_BodyPart_content_type
{
public:
  explicit Init_BodyPart_content_type(::ros2web_interfaces::msg::BodyPart & msg)
  : msg_(msg)
  {}
  Init_BodyPart_headers content_type(::ros2web_interfaces::msg::BodyPart::_content_type_type arg)
  {
    msg_.content_type = std::move(arg);
    return Init_BodyPart_headers(msg_);
  }

private:
  ::ros2web_interfaces::msg::BodyPart msg_;
};

class Init_BodyPart_filename
{
public:
  explicit Init_BodyPart_filename(::ros2web_interfaces::msg::BodyPart & msg)
  : msg_(msg)
  {}
  Init_BodyPart_content_type filename(::ros2web_interfaces::msg::BodyPart::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return Init_BodyPart_content_type(msg_);
  }

private:
  ::ros2web_interfaces::msg::BodyPart msg_;
};

class Init_BodyPart_name
{
public:
  Init_BodyPart_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyPart_filename name(::ros2web_interfaces::msg::BodyPart::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_BodyPart_filename(msg_);
  }

private:
  ::ros2web_interfaces::msg::BodyPart msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2web_interfaces::msg::BodyPart>()
{
  return ros2web_interfaces::msg::builder::Init_BodyPart_name();
}

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__BUILDER_HPP_
