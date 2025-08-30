// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:msg/WSMsgData.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__STRUCT_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__msg__WSMsgData __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__msg__WSMsgData __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WSMsgData_
{
  using Type = WSMsgData_<ContainerAllocator>;

  explicit WSMsgData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->str = "";
    }
  }

  explicit WSMsgData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : str(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->str = "";
    }
  }

  // field types and members
  using _str_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _str_type str;
  using _bytes_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _bytes_type bytes;

  // setters for named parameter idiom
  Type & set__str(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->str = _arg;
    return *this;
  }
  Type & set__bytes(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->bytes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__msg__WSMsgData
    std::shared_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__msg__WSMsgData
    std::shared_ptr<ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WSMsgData_ & other) const
  {
    if (this->str != other.str) {
      return false;
    }
    if (this->bytes != other.bytes) {
      return false;
    }
    return true;
  }
  bool operator!=(const WSMsgData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WSMsgData_

// alias to use template instance with default allocator
using WSMsgData =
  ros2web_interfaces::msg::WSMsgData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_DATA__STRUCT_HPP_
