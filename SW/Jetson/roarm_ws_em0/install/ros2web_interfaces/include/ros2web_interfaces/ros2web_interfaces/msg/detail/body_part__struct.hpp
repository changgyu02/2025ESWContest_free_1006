// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:msg/BodyPart.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__STRUCT_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__msg__BodyPart __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__msg__BodyPart __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BodyPart_
{
  using Type = BodyPart_<ContainerAllocator>;

  explicit BodyPart_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->filename = "";
      this->content_type = "";
    }
  }

  explicit BodyPart_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    filename(_alloc),
    content_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->filename = "";
      this->content_type = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;
  using _content_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _content_type_type content_type;
  using _headers_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _headers_type headers;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }
  Type & set__content_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->content_type = _arg;
    return *this;
  }
  Type & set__headers(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->headers = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2web_interfaces::msg::BodyPart_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::msg::BodyPart_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__msg__BodyPart
    std::shared_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__msg__BodyPart
    std::shared_ptr<ros2web_interfaces::msg::BodyPart_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BodyPart_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->filename != other.filename) {
      return false;
    }
    if (this->content_type != other.content_type) {
      return false;
    }
    if (this->headers != other.headers) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const BodyPart_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BodyPart_

// alias to use template instance with default allocator
using BodyPart =
  ros2web_interfaces::msg::BodyPart_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__BODY_PART__STRUCT_HPP_
