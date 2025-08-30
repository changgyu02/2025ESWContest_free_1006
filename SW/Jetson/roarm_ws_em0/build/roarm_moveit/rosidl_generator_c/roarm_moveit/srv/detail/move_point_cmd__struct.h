// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roarm_moveit:srv/MovePointCmd.idl
// generated code does not contain a copyright notice

#ifndef ROARM_MOVEIT__SRV__DETAIL__MOVE_POINT_CMD__STRUCT_H_
#define ROARM_MOVEIT__SRV__DETAIL__MOVE_POINT_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MovePointCmd in the package roarm_moveit.
typedef struct roarm_moveit__srv__MovePointCmd_Request
{
  /// 목표 위치의 x 좌표
  double x;
  /// 목표 위치의 y 좌표
  double y;
  /// 목표 위치의 z 좌표
  double z;
} roarm_moveit__srv__MovePointCmd_Request;

// Struct for a sequence of roarm_moveit__srv__MovePointCmd_Request.
typedef struct roarm_moveit__srv__MovePointCmd_Request__Sequence
{
  roarm_moveit__srv__MovePointCmd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_moveit__srv__MovePointCmd_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MovePointCmd in the package roarm_moveit.
typedef struct roarm_moveit__srv__MovePointCmd_Response
{
  /// 실행 성공 여부를 나타내는 플래그
  bool success;
  /// 실행 결과 메시지
  rosidl_runtime_c__String message;
} roarm_moveit__srv__MovePointCmd_Response;

// Struct for a sequence of roarm_moveit__srv__MovePointCmd_Response.
typedef struct roarm_moveit__srv__MovePointCmd_Response__Sequence
{
  roarm_moveit__srv__MovePointCmd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_moveit__srv__MovePointCmd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROARM_MOVEIT__SRV__DETAIL__MOVE_POINT_CMD__STRUCT_H_
