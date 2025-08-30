// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:msg/ContentType.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__CONTENT_TYPE__STRUCT_HPP_
#define ROS2WEB_INTERFACES__MSG__DETAIL__CONTENT_TYPE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__msg__ContentType __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__msg__ContentType __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ContentType_
{
  using Type = ContentType_<ContainerAllocator>;

  explicit ContentType_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ContentType_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_JSON;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_MSWORD;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_MSWORD_OPEN_XML;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_OCTET_STREAM;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_VND_AMAZON_EBOOK;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_X_ABIWORD;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_X_BZIP2;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_X_BZIP;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_X_CSH;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_X_FREEARC;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> APPLICATION_X_WWW_FORM_URLENCODED;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> AUDIO_AAC;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> IMAGE_BMP;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> IMAGE_JPEG;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> IMAGE_PNG;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MULTIPART_FORM_DATA;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TEXT_CSS;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TEXT_CSV;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TEXT_HTML;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TEXT_JAVASCRIPT;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TEXT_PLAIN;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TEXT_XML;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> VIDEO_X_MSVIDEO;

  // pointer types
  using RawPtr =
    ros2web_interfaces::msg::ContentType_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::msg::ContentType_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::ContentType_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::msg::ContentType_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__msg__ContentType
    std::shared_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__msg__ContentType
    std::shared_ptr<ros2web_interfaces::msg::ContentType_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContentType_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContentType_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContentType_

// alias to use template instance with default allocator
using ContentType =
  ros2web_interfaces::msg::ContentType_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_JSON = "application/json";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_MSWORD = "application/msword";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_MSWORD_OPEN_XML = "application/vnd.openxmlformats-officedocument.wordprocessingml.document";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_OCTET_STREAM = "application/octet-stream";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_VND_AMAZON_EBOOK = "application/vnd.amazon.ebook";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_X_ABIWORD = "application/x-abiword";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_X_BZIP2 = "application/x-bzip2";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_X_BZIP = "application/x-bzip";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_X_CSH = "application/x-csh";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_X_FREEARC = "application/x-freearc";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::APPLICATION_X_WWW_FORM_URLENCODED = "application/x-www-form-urlencoded";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::AUDIO_AAC = "audio/aac";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::IMAGE_BMP = "image/bmp";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::IMAGE_JPEG = "image/jpeg";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::IMAGE_PNG = "image/png";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::MULTIPART_FORM_DATA = "multipart/form-data";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::TEXT_CSS = "text/css";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::TEXT_CSV = "text/csv";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::TEXT_HTML = "text/html";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::TEXT_JAVASCRIPT = "text/javascript";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::TEXT_PLAIN = "text/plain";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::TEXT_XML = "text/xml";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
ContentType_<ContainerAllocator>::VIDEO_X_MSVIDEO = "video/x-msvideo";

}  // namespace msg

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__CONTENT_TYPE__STRUCT_HPP_
