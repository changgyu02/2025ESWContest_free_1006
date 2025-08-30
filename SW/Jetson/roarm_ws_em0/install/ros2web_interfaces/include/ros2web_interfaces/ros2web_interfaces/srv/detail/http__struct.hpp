// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__STRUCT_HPP_
#define ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'multipart'
#include "ros2web_interfaces/msg/detail/body_part__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__srv__HTTP_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__srv__HTTP_Request __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HTTP_Request_
{
  using Type = HTTP_Request_<ContainerAllocator>;

  explicit HTTP_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->method = "";
      this->path = "";
      this->query = "";
      this->srv_name = "";
      this->route = "";
      this->text = "";
      this->content_type = "";
      this->charset = "";
    }
  }

  explicit HTTP_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : method(_alloc),
    path(_alloc),
    query(_alloc),
    srv_name(_alloc),
    route(_alloc),
    text(_alloc),
    content_type(_alloc),
    charset(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->method = "";
      this->path = "";
      this->query = "";
      this->srv_name = "";
      this->route = "";
      this->text = "";
      this->content_type = "";
      this->charset = "";
    }
  }

  // field types and members
  using _headers_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _headers_type headers;
  using _method_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _method_type method;
  using _path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _path_type path;
  using _query_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _query_type query;
  using _srv_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _srv_name_type srv_name;
  using _route_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _route_type route;
  using _body_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _body_type body;
  using _text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _text_type text;
  using _content_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _content_type_type content_type;
  using _charset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _charset_type charset;
  using _multipart_type =
    std::vector<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>>;
  _multipart_type multipart;

  // setters for named parameter idiom
  Type & set__headers(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->headers = _arg;
    return *this;
  }
  Type & set__method(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->method = _arg;
    return *this;
  }
  Type & set__path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->path = _arg;
    return *this;
  }
  Type & set__query(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->query = _arg;
    return *this;
  }
  Type & set__srv_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->srv_name = _arg;
    return *this;
  }
  Type & set__route(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->route = _arg;
    return *this;
  }
  Type & set__body(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->body = _arg;
    return *this;
  }
  Type & set__text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->text = _arg;
    return *this;
  }
  Type & set__content_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->content_type = _arg;
    return *this;
  }
  Type & set__charset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->charset = _arg;
    return *this;
  }
  Type & set__multipart(
    const std::vector<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2web_interfaces::msg::BodyPart_<ContainerAllocator>>> & _arg)
  {
    this->multipart = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__srv__HTTP_Request
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__srv__HTTP_Request
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HTTP_Request_ & other) const
  {
    if (this->headers != other.headers) {
      return false;
    }
    if (this->method != other.method) {
      return false;
    }
    if (this->path != other.path) {
      return false;
    }
    if (this->query != other.query) {
      return false;
    }
    if (this->srv_name != other.srv_name) {
      return false;
    }
    if (this->route != other.route) {
      return false;
    }
    if (this->body != other.body) {
      return false;
    }
    if (this->text != other.text) {
      return false;
    }
    if (this->content_type != other.content_type) {
      return false;
    }
    if (this->charset != other.charset) {
      return false;
    }
    if (this->multipart != other.multipart) {
      return false;
    }
    return true;
  }
  bool operator!=(const HTTP_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HTTP_Request_

// alias to use template instance with default allocator
using HTTP_Request =
  ros2web_interfaces::srv::HTTP_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2web_interfaces


#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__srv__HTTP_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__srv__HTTP_Response __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HTTP_Response_
{
  using Type = HTTP_Response_<ContainerAllocator>;

  explicit HTTP_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->reason = "";
      this->text = "";
      this->content_type = "";
      this->charset = "";
      this->file_path = "";
    }
  }

  explicit HTTP_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reason(_alloc),
    text(_alloc),
    content_type(_alloc),
    charset(_alloc),
    file_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->reason = "";
      this->text = "";
      this->content_type = "";
      this->charset = "";
      this->file_path = "";
    }
  }

  // field types and members
  using _status_type =
    uint16_t;
  _status_type status;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _body_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _body_type body;
  using _text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _text_type text;
  using _content_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _content_type_type content_type;
  using _charset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _charset_type charset;
  using _file_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_path_type file_path;

  // setters for named parameter idiom
  Type & set__status(
    const uint16_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__body(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->body = _arg;
    return *this;
  }
  Type & set__text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->text = _arg;
    return *this;
  }
  Type & set__content_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->content_type = _arg;
    return *this;
  }
  Type & set__charset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->charset = _arg;
    return *this;
  }
  Type & set__file_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__srv__HTTP_Response
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__srv__HTTP_Response
    std::shared_ptr<ros2web_interfaces::srv::HTTP_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HTTP_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->body != other.body) {
      return false;
    }
    if (this->text != other.text) {
      return false;
    }
    if (this->content_type != other.content_type) {
      return false;
    }
    if (this->charset != other.charset) {
      return false;
    }
    if (this->file_path != other.file_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const HTTP_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HTTP_Response_

// alias to use template instance with default allocator
using HTTP_Response =
  ros2web_interfaces::srv::HTTP_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2web_interfaces

namespace ros2web_interfaces
{

namespace srv
{

struct HTTP
{
  using Request = ros2web_interfaces::srv::HTTP_Request;
  using Response = ros2web_interfaces::srv::HTTP_Response;
};

}  // namespace srv

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__HTTP__STRUCT_HPP_
