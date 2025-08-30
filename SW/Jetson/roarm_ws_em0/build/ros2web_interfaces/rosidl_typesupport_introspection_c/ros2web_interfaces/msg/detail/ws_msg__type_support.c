// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2web_interfaces/msg/detail/ws_msg__rosidl_typesupport_introspection_c.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2web_interfaces/msg/detail/ws_msg__functions.h"
#include "ros2web_interfaces/msg/detail/ws_msg__struct.h"


// Include directives for member types
// Member `route`
// Member `ws_id`
// Member `extra`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "ros2web_interfaces/msg/ws_msg_data.h"
// Member `data`
#include "ros2web_interfaces/msg/detail/ws_msg_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2web_interfaces__msg__WSMsg__init(message_memory);
}

void ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_fini_function(void * message_memory)
{
  ros2web_interfaces__msg__WSMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_member_array[5] = {
  {
    "route",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__WSMsg, route),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ws_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__WSMsg, ws_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__WSMsg, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__WSMsg, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "extra",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__msg__WSMsg, extra),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_members = {
  "ros2web_interfaces__msg",  // message namespace
  "WSMsg",  // message name
  5,  // number of fields
  sizeof(ros2web_interfaces__msg__WSMsg),
  ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_member_array,  // message members
  ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_type_support_handle = {
  0,
  &ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, msg, WSMsg)() {
  ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, msg, WSMsgData)();
  if (!ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2web_interfaces__msg__WSMsg__rosidl_typesupport_introspection_c__WSMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
