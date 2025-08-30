// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2web_interfaces:srv/WS.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__SRV__DETAIL__WS__STRUCT_HPP_
#define ROS2WEB_INTERFACES__SRV__DETAIL__WS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'data'
#include "ros2web_interfaces/msg/detail/ws_msg_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__srv__WS_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__srv__WS_Request __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WS_Request_
{
  using Type = WS_Request_<ContainerAllocator>;

  explicit WS_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_name = "";
      this->route = "";
      this->type = 0;
      this->ws_id = "";
      this->extra = "";
    }
  }

  explicit WS_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : srv_name(_alloc),
    route(_alloc),
    ws_id(_alloc),
    data(_alloc, _init),
    extra(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_name = "";
      this->route = "";
      this->type = 0;
      this->ws_id = "";
      this->extra = "";
    }
  }

  // field types and members
  using _srv_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _srv_name_type srv_name;
  using _route_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _route_type route;
  using _type_type =
    uint8_t;
  _type_type type;
  using _ws_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ws_id_type ws_id;
  using _data_type =
    ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>;
  _data_type data;
  using _extra_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _extra_type extra;

  // setters for named parameter idiom
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
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__ws_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ws_id = _arg;
    return *this;
  }
  Type & set__data(
    const ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__extra(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->extra = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2web_interfaces::srv::WS_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::srv::WS_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::WS_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::WS_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__srv__WS_Request
    std::shared_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__srv__WS_Request
    std::shared_ptr<ros2web_interfaces::srv::WS_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WS_Request_ & other) const
  {
    if (this->srv_name != other.srv_name) {
      return false;
    }
    if (this->route != other.route) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->ws_id != other.ws_id) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->extra != other.extra) {
      return false;
    }
    return true;
  }
  bool operator!=(const WS_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WS_Request_

// alias to use template instance with default allocator
using WS_Request =
  ros2web_interfaces::srv::WS_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2web_interfaces


// Include directives for member types
// Member 'data'
// already included above
// #include "ros2web_interfaces/msg/detail/ws_msg_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2web_interfaces__srv__WS_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2web_interfaces__srv__WS_Response __declspec(deprecated)
#endif

namespace ros2web_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WS_Response_
{
  using Type = WS_Response_<ContainerAllocator>;

  explicit WS_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->ws_id = "";
      this->extra = "";
    }
  }

  explicit WS_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ws_id(_alloc),
    data(_alloc, _init),
    extra(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->ws_id = "";
      this->extra = "";
    }
  }

  // field types and members
  using _type_type =
    uint8_t;
  _type_type type;
  using _ws_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ws_id_type ws_id;
  using _data_type =
    ros2web_interfaces::msg::WSMsgData_<ContainerAllocator>;
  _data_type data;
  using _extra_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _extra_type extra;

  // setters for named parameter idiom
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__ws_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ws_id = _arg;
    return *this;
  }
  Type & set__data(
    const ros2web_interfaces::msg::WSMsgData_<ContainerAllocator> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__extra(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->extra = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2web_interfaces::srv::WS_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2web_interfaces::srv::WS_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::WS_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2web_interfaces::srv::WS_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2web_interfaces__srv__WS_Response
    std::shared_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2web_interfaces__srv__WS_Response
    std::shared_ptr<ros2web_interfaces::srv::WS_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WS_Response_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->ws_id != other.ws_id) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->extra != other.extra) {
      return false;
    }
    return true;
  }
  bool operator!=(const WS_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WS_Response_

// alias to use template instance with default allocator
using WS_Response =
  ros2web_interfaces::srv::WS_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2web_interfaces

namespace ros2web_interfaces
{

namespace srv
{

struct WS
{
  using Request = ros2web_interfaces::srv::WS_Request;
  using Response = ros2web_interfaces::srv::WS_Response;
};

}  // namespace srv

}  // namespace ros2web_interfaces

#endif  // ROS2WEB_INTERFACES__SRV__DETAIL__WS__STRUCT_HPP_
