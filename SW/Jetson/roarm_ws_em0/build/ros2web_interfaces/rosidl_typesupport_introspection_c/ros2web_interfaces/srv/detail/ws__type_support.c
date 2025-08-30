// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2web_interfaces:srv/WS.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2web_interfaces/srv/detail/ws__rosidl_typesupport_introspection_c.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2web_interfaces/srv/detail/ws__functions.h"
#include "ros2web_interfaces/srv/detail/ws__struct.h"


// Include directives for member types
// Member `srv_name`
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

void ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2web_interfaces__srv__WS_Request__init(message_memory);
}

void ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_fini_function(void * message_memory)
{
  ros2web_interfaces__srv__WS_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_member_array[6] = {
  {
    "srv_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__WS_Request, srv_name),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Request, route),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Request, type),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Request, ws_id),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Request, data),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Request, extra),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_members = {
  "ros2web_interfaces__srv",  // message namespace
  "WS_Request",  // message name
  6,  // number of fields
  sizeof(ros2web_interfaces__srv__WS_Request),
  ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_member_array,  // message members
  ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_type_support_handle = {
  0,
  &ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS_Request)() {
  ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, msg, WSMsgData)();
  if (!ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2web_interfaces__srv__WS_Request__rosidl_typesupport_introspection_c__WS_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2web_interfaces/srv/detail/ws__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2web_interfaces/srv/detail/ws__functions.h"
// already included above
// #include "ros2web_interfaces/srv/detail/ws__struct.h"


// Include directives for member types
// Member `ws_id`
// Member `extra`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `data`
// already included above
// #include "ros2web_interfaces/msg/ws_msg_data.h"
// Member `data`
// already included above
// #include "ros2web_interfaces/msg/detail/ws_msg_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2web_interfaces__srv__WS_Response__init(message_memory);
}

void ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_fini_function(void * message_memory)
{
  ros2web_interfaces__srv__WS_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_member_array[4] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2web_interfaces__srv__WS_Response, type),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Response, ws_id),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Response, data),  // bytes offset in struct
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
    offsetof(ros2web_interfaces__srv__WS_Response, extra),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_members = {
  "ros2web_interfaces__srv",  // message namespace
  "WS_Response",  // message name
  4,  // number of fields
  sizeof(ros2web_interfaces__srv__WS_Response),
  ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_member_array,  // message members
  ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_type_support_handle = {
  0,
  &ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS_Response)() {
  ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, msg, WSMsgData)();
  if (!ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2web_interfaces__srv__WS_Response__rosidl_typesupport_introspection_c__WS_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ros2web_interfaces/srv/detail/ws__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_members = {
  "ros2web_interfaces__srv",  // service namespace
  "WS",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_Request_message_type_support_handle,
  NULL  // response message
  // ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_Response_message_type_support_handle
};

static rosidl_service_type_support_t ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_type_support_handle = {
  0,
  &ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2web_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS)() {
  if (!ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_type_support_handle.typesupport_identifier) {
    ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2web_interfaces, srv, WS_Response)()->data;
  }

  return &ros2web_interfaces__srv__detail__ws__rosidl_typesupport_introspection_c__WS_service_type_support_handle;
}
