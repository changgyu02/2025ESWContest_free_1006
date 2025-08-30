// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2web_interfaces:srv/HTTP.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2web_interfaces/srv/detail/http__rosidl_typesupport_introspection_c.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2web_interfaces/srv/detail/http__functions.h"
#include "ros2web_interfaces/srv/detail/http__struct.h"


// Include directives for member types
// Member `headers`
// Member `method`
// Member `path`
// Member `query`
// Member `srv_name`
// Member `route`
// Member `text`
// Member `content_type`
// Member `charset`
#include "rosidl_runtime_c/string_functions.h"
// Member `body`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `multipart`
#include "ros2web_interfaces/msg/body_part.h"
// Member `multipart`
#include "ros2web_interfaces/msg/detail/body_part__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2web_interfaces__srv__HTTP_Request__init(message_memory);
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_fini_function(void * message_memory)
{
  ros2web_interfaces__srv__HTTP_Request__fini(message_memory);
}

size_t ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__size_function__HTTP_Request__headers(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__headers(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__headers(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__fetch_function__HTTP_Request__headers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__headers(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__assign_function__HTTP_Request__headers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__headers(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__resize_function__HTTP_Request__headers(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__size_function__HTTP_Request__body(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__body(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__body(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__fetch_function__HTTP_Request__body(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__body(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__assign_function__HTTP_Request__body(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__body(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__resize_function__HTTP_Request__body(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__size_function__HTTP_Request__multipart(
  const void * untyped_member)
{
  const ros2web_interfaces__msg__BodyPart__Sequence * member =
    (const ros2web_interfaces__msg__BodyPart__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__multipart(
  const void * untyped_member, size_t index)
{
  const ros2web_interfaces__msg__BodyPart__Sequence * member =
    (const ros2web_interfaces__msg__BodyPart__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__multipart(
  void * untyped_member, size_t index)
{
  ros2web_interfaces__msg__BodyPart__Sequence * member =
    (ros2web_interfaces__msg__BodyPart__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__fetch_function__HTTP_Request__multipart(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2web_interfaces__msg__BodyPart * item =
    ((const ros2web_interfaces__msg__BodyPart *)
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__multipart(untyped_member, index));
  ros2web_interfaces__msg__BodyPart * value =
    (ros2web_interfaces__msg__BodyPart *)(untyped_value);
  *value = *item;
}

void ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__assign_function__HTTP_Request__multipart(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2web_interfaces__msg__BodyPart * item =
    ((ros2web_interfaces__msg__BodyPart *)
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__multipart(untyped_member, index));
  const ros2web_interfaces__msg__BodyPart * value =
    (const ros2web_interfaces__msg__BodyPart *)(untyped_value);
  *item = *value;
}

bool ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__resize_function__HTTP_Request__multipart(
  void * untyped_member, size_t size)
{
  ros2web_interfaces__msg__BodyPart__Sequence * member =
    (ros2web_interfaces__msg__BodyPart__Sequence *)(untyped_member);
  ros2web_interfaces__msg__BodyPart__Sequence__fini(member);
  return ros2web_interfaces__msg__BodyPart__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_member_array[11] = {
  {
    "headers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, headers),  // bytes offset in struct
    NULL,  // default value
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__size_function__HTTP_Request__headers,  // size() function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__headers,  // get_const(index) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__headers,  // get(index) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__fetch_function__HTTP_Request__headers,  // fetch(index, &value) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__assign_function__HTTP_Request__headers,  // assign(index, value) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__resize_function__HTTP_Request__headers  // resize(index) function pointer
  },
  {
    "method",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, method),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "query",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, query),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "srv_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, srv_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "route",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, route),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, body),  // bytes offset in struct
    NULL,  // default value
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__size_function__HTTP_Request__body,  // size() function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__body,  // get_const(index) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__body,  // get(index) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__fetch_function__HTTP_Request__body,  // fetch(index, &value) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__assign_function__HTTP_Request__body,  // assign(index, value) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__resize_function__HTTP_Request__body  // resize(index) function pointer
  },
  {
    "text",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, text),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "content_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, content_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "charset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, charset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "multipart",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Request, multipart),  // bytes offset in struct
    NULL,  // default value
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__size_function__HTTP_Request__multipart,  // size() function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_const_function__HTTP_Request__multipart,  // get_const(index) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__get_function__HTTP_Request__multipart,  // get(index) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__fetch_function__HTTP_Request__multipart,  // fetch(index, &value) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__assign_function__HTTP_Request__multipart,  // assign(index, value) function pointer
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__resize_function__HTTP_Request__multipart  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_members = {
  "ros2web_interfaces__srv",  // message namespace
  "HTTP_Request",  // message name
  11,  // number of fields
  sizeof(ros2web_interfaces__srv__HTTP_Request),
  ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_member_array,  // message members
  ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_type_support_handle = {
  0,
  &ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP_Request)() {
  ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, msg, BodyPart)();
  if (!ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2web_interfaces__srv__HTTP_Request__rosidl_typesupport_introspection_c__HTTP_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2web_interfaces/srv/detail/http__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2web_interfaces/srv/detail/http__functions.h"
// already included above
// #include "ros2web_interfaces/srv/detail/http__struct.h"


// Include directives for member types
// Member `reason`
// Member `text`
// Member `content_type`
// Member `charset`
// Member `file_path`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `body`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2web_interfaces__srv__HTTP_Response__init(message_memory);
}

void ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_fini_function(void * message_memory)
{
  ros2web_interfaces__srv__HTTP_Response__fini(message_memory);
}

size_t ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__size_function__HTTP_Response__body(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__get_const_function__HTTP_Response__body(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__get_function__HTTP_Response__body(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__fetch_function__HTTP_Response__body(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__get_const_function__HTTP_Response__body(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__assign_function__HTTP_Response__body(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__get_function__HTTP_Response__body(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__resize_function__HTTP_Response__body(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_member_array[7] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, body),  // bytes offset in struct
    NULL,  // default value
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__size_function__HTTP_Response__body,  // size() function pointer
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__get_const_function__HTTP_Response__body,  // get_const(index) function pointer
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__get_function__HTTP_Response__body,  // get(index) function pointer
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__fetch_function__HTTP_Response__body,  // fetch(index, &value) function pointer
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__assign_function__HTTP_Response__body,  // assign(index, value) function pointer
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__resize_function__HTTP_Response__body  // resize(index) function pointer
  },
  {
    "text",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, text),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "content_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, content_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "charset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, charset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "file_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__HTTP_Response, file_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_members = {
  "ros2web_interfaces__srv",  // message namespace
  "HTTP_Response",  // message name
  7,  // number of fields
  sizeof(ros2web_interfaces__srv__HTTP_Response),
  ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_member_array,  // message members
  ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_type_support_handle = {
  0,
  &ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP_Response)() {
  if (!ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2web_interfaces__srv__HTTP_Response__rosidl_typesupport_introspection_c__HTTP_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ros2web_interfaces/srv/detail/http__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_members = {
  "ros2web_interfaces__srv",  // service namespace
  "HTTP",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_Request_message_type_support_handle,
  NULL  // response message
  // ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_Response_message_type_support_handle
};

static rosidl_service_type_support_t ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_type_support_handle = {
  0,
  &ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP)() {
  if (!ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, HTTP_Response)()->data;
  }

  return &ros2web_interfaces__srv__detail__http__rosidl_typesupport_introspection_c__HTTP_service_type_support_handle;
}
