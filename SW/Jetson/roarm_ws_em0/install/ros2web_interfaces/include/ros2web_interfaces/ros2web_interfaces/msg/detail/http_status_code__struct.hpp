// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:msg/HTTPStatusCode.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__HTTP_STATUS_CODE__STRUCT_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__HTTP_STATUS_CODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__msg__HTTPStatusCode __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__msg__HTTPStatusCode __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HTTPStatusCode_
{
  using Type = HTTPStatusCode_<ContainerAllocator>;

  explicit HTTPStatusCode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit HTTPStatusCode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  static constexpr uint16_t HTTP_OK =
    200u;
  static constexpr uint16_t HTTP_ACCEPTED =
    202u;
  static constexpr uint16_t HTTP_NON_AUTHORITATIVE_INFORMATION =
    203u;
  static constexpr uint16_t HTTP_NO_CONTENT =
    204u;
  static constexpr uint16_t HTTP_RESET_CONTENT =
    205u;
  static constexpr uint16_t HTTP_PARTIAL_CONTENT =
    206u;
  static constexpr uint16_t HTTP_MULTIPLE_CHOICES =
    300u;
  static constexpr uint16_t HTTP_MOVED_PERMANENTLY =
    301u;
  static constexpr uint16_t HTTP_FOUND =
    302u;
  static constexpr uint16_t HTTP_SEE_OTHER =
    303u;
  static constexpr uint16_t HTTP_NOT_MODIFIED =
    304u;
  static constexpr uint16_t HTTP_USE_PROXY =
    305u;
  static constexpr uint16_t HTTP_TEMPORARY_REDIRECT =
    307u;
  static constexpr uint16_t HTTP_PERMANENT_REDIRECT =
    308u;
  static constexpr uint16_t HTTP_BAD_REQUEST =
    400u;
  static constexpr uint16_t HTTP_UNAUTHORIZED =
    401u;
  static constexpr uint16_t HTTP_PAYMENT_REQUIRED =
    402u;
  static constexpr uint16_t HTTP_FORBIDDEN =
    403u;
  static constexpr uint16_t HTTP_NOT_FOUND =
    404u;
  static constexpr uint16_t HTTP_METHOD_NOT_ALLOWED =
    405u;
  static constexpr uint16_t HTTP_NOT_ACCEPTABLE =
    406u;
  static constexpr uint16_t HTTP_PROXY_AUTHENTICATION_REQUIRED =
    407u;
  static constexpr uint16_t HTTP_REQUEST_TIMEOUT =
    408u;
  static constexpr uint16_t HTTP_CONFLICT =
    409u;
  static constexpr uint16_t HTTP_GONE =
    410u;
  static constexpr uint16_t HTTP_LENGTH_REQUIRED =
    411u;
  static constexpr uint16_t HTTP_PRECONDITION_FAILED =
    412u;
  static constexpr uint16_t HTTP_REQUEST_ENTITY_TOO_LARGE =
    413u;
  static constexpr uint16_t HTTP_REQUEST_URI_TOOLONG =
    414u;
  static constexpr uint16_t HTTP_UNSUPPORTED_MEDIA_TYPE =
    415u;
  static constexpr uint16_t HTTP_REQUEST_RANGE_NOT_SATISFIABLE =
    416u;
  static constexpr uint16_t HTTP_EXPECTATION_FAILED =
    417u;
  static constexpr uint16_t HTTP_MISDIRECTED_REQUEST =
    421u;
  static constexpr uint16_t HTTP_UNPROCESSABLE_ENTITY =
    422u;
  static constexpr uint16_t HTTP_FAILED_DEPENDENCY =
    424u;
  static constexpr uint16_t HTTP_UPGRADE_REQUIRED =
    426u;
  static constexpr uint16_t HTTP_PRECONDITION_REQUIRED =
    428u;
  static constexpr uint16_t HTTP_TOO_MANY_REQUESTS =
    429u;
  static constexpr uint16_t HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE =
    431u;
  static constexpr uint16_t HTTP_UNAVAILABLE_FOR_LEGAL_REASONS =
    451u;
  static constexpr uint16_t HTTP_INTERNAL_SERVER_ERROR =
    500u;
  static constexpr uint16_t HTTP_NOT_IMPLEMENTED =
    501u;
  static constexpr uint16_t HTTP_BAD_GATEWAY =
    502u;
  static constexpr uint16_t HTTP_SERVICE_UNAVAILABLE =
    503u;
  static constexpr uint16_t HTTP_GATEWAY_TIMEOUT =
    504u;
  static constexpr uint16_t HTTP_VERSION_NOT_SUPPORTED =
    505u;
  static constexpr uint16_t HTTP_VARIANT_ALSO_NEGOTIATES =
    506u;
  static constexpr uint16_t HTTP_INSUFFICIENT_STORAGE =
    507u;
  static constexpr uint16_t HTTP_NOT_EXTENDED =
    510u;
  static constexpr uint16_t HTTP_NETWORK_AUTHENTICATION_REQUIRED =
    511u;

  // pointer types
  using RawPtr =
    ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__msg__HTTPStatusCode
    std::shared_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__msg__HTTPStatusCode
    std::shared_ptr<ros2web_interfaces::msg::HTTPStatusCode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HTTPStatusCode_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const HTTPStatusCode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HTTPStatusCode_

// alias to use template instance with default allocator
using HTTPStatusCode =
  ros2web_interfaces::msg::HTTPStatusCode_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_ACCEPTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NON_AUTHORITATIVE_INFORMATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NO_CONTENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_RESET_CONTENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_PARTIAL_CONTENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_MULTIPLE_CHOICES;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_MOVED_PERMANENTLY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_FOUND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_SEE_OTHER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NOT_MODIFIED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_USE_PROXY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_TEMPORARY_REDIRECT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_PERMANENT_REDIRECT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_BAD_REQUEST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_UNAUTHORIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_PAYMENT_REQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_FORBIDDEN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NOT_FOUND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_METHOD_NOT_ALLOWED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NOT_ACCEPTABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_PROXY_AUTHENTICATION_REQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_REQUEST_TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_CONFLICT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_GONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_LENGTH_REQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_PRECONDITION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_REQUEST_ENTITY_TOO_LARGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_REQUEST_URI_TOOLONG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_UNSUPPORTED_MEDIA_TYPE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_REQUEST_RANGE_NOT_SATISFIABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_EXPECTATION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_MISDIRECTED_REQUEST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_UNPROCESSABLE_ENTITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_FAILED_DEPENDENCY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_UPGRADE_REQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_PRECONDITION_REQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_TOO_MANY_REQUESTS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_UNAVAILABLE_FOR_LEGAL_REASONS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_INTERNAL_SERVER_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NOT_IMPLEMENTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_BAD_GATEWAY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_SERVICE_UNAVAILABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_GATEWAY_TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_VERSION_NOT_SUPPORTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_VARIANT_ALSO_NEGOTIATES;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_INSUFFICIENT_STORAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NOT_EXTENDED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t HTTPStatusCode_<ContainerAllocator>::HTTP_NETWORK_AUTHENTICATION_REQUIRED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__HTTP_STATUS_CODE__STRUCT_HPP_
