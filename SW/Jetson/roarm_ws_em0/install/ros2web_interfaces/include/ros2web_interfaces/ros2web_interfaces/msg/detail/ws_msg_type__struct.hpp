// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:msg/WSMsgType.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__STRUCT_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__msg__WSMsgType __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__msg__WSMsgType __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WSMsgType_
{
  using Type = WSMsgType_<ContainerAllocator>;

  explicit WSMsgType_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit WSMsgType_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations
  static constexpr uint8_t CONTINUATION =
    0u;
  static constexpr uint8_t TEXT =
    1u;
  static constexpr uint8_t BINARY =
    2u;
  static constexpr uint8_t PING =
    11u;
  static constexpr uint8_t PONG =
    12u;
  static constexpr uint8_t CLOSE =
    21u;
  static constexpr uint8_t CLOSING =
    22u;
  static constexpr uint8_t CLOSED =
    23u;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr uint8_t ERROR =
    31u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif

  // pointer types
  using RawPtr =
    ros2web_interfaces::msg::WSMsgType_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::msg::WSMsgType_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::WSMsgType_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::WSMsgType_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__msg__WSMsgType
    std::shared_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__msg__WSMsgType
    std::shared_ptr<ros2web_interfaces::msg::WSMsgType_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WSMsgType_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const WSMsgType_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WSMsgType_

// alias to use template instance with default allocator
using WSMsgType =
  ros2web_interfaces::msg::WSMsgType_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::CONTINUATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::TEXT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::BINARY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::PING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::PONG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::CLOSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::CLOSING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::CLOSED;
#endif  // __cplusplus < 201703L
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t WSMsgType_<ContainerAllocator>::ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif

}  // namespace msg

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG_TYPE__STRUCT_HPP_
