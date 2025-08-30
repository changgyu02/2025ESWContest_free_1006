// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2web_interfaces:msg/BodyPart.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2web_interfaces/msg/detail/body_part__rosidl_typesupport_introspection_c.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2web_interfaces/msg/detail/body_part__functions.h"
#include "ros2web_interfaces/msg/detail/body_part__struct.h"


// Include directives for member types
// Member `name`
// Member `filename`
// Member `content_type`
// Member `headers`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2web_interfaces__msg__BodyPart__init(message_memory);
}

void ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_fini_function(void * message_memory)
{
  ros2web_interfaces__msg__BodyPart__fini(message_memory);
}

size_t ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__size_function__BodyPart__headers(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_const_function__BodyPart__headers(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_function__BodyPart__headers(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__fetch_function__BodyPart__headers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_const_function__BodyPart__headers(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__assign_function__BodyPart__headers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_function__BodyPart__headers(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__resize_function__BodyPart__headers(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__size_function__BodyPart__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_const_function__BodyPart__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_function__BodyPart__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__fetch_function__BodyPart__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_const_function__BodyPart__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__assign_function__BodyPart__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_function__BodyPart__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__resize_function__BodyPart__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_member_array[5] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__BodyPart, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__BodyPart, filename),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__msg__BodyPart, content_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "headers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__BodyPart, headers),  // bytes offset in struct
    NULL,  // default value
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__size_function__BodyPart__headers,  // size() function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_const_function__BodyPart__headers,  // get_const(index) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_function__BodyPart__headers,  // get(index) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__fetch_function__BodyPart__headers,  // fetch(index, &value) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__assign_function__BodyPart__headers,  // assign(index, value) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__resize_function__BodyPart__headers  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__BodyPart, data),  // bytes offset in struct
    NULL,  // default value
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__size_function__BodyPart__data,  // size() function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_const_function__BodyPart__data,  // get_const(index) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__get_function__BodyPart__data,  // get(index) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__fetch_function__BodyPart__data,  // fetch(index, &value) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__assign_function__BodyPart__data,  // assign(index, value) function pointer
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__resize_function__BodyPart__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_members = {
  "ros2web_interfaces__msg",  // message namespace
  "BodyPart",  // message name
  5,  // number of fields
  sizeof(ros2web_interfaces__msg__BodyPart),
  ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_member_array,  // message members
  ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_type_support_handle = {
  0,
  &ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, msg, BodyPart)() {
  if (!ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2web_interfaces__msg__BodyPart__rosidl_typesupport_introspection_c__BodyPart_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
