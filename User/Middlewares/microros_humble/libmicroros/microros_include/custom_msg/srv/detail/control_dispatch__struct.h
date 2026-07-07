// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:srv/ControlDispatch.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__SRV__DETAIL__CONTROL_DISPATCH__STRUCT_H_
#define CUSTOM_MSG__SRV__DETAIL__CONTROL_DISPATCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/ControlDispatch in the package custom_msg.
typedef struct custom_msg__srv__ControlDispatch_Request
{
  /// 动作指令：0=准备, 1=抓取, 2=检查, 3=拼接
  int8_t command_mode;
  int8_t event;
  geometry_msgs__msg__Point point;
  bool is_return;
} custom_msg__srv__ControlDispatch_Request;

// Struct for a sequence of custom_msg__srv__ControlDispatch_Request.
typedef struct custom_msg__srv__ControlDispatch_Request__Sequence
{
  custom_msg__srv__ControlDispatch_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__srv__ControlDispatch_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ControlDispatch in the package custom_msg.
typedef struct custom_msg__srv__ControlDispatch_Response
{
  /// 动作是否成功？(true/false)
  bool success;
  /// 附加的文字信息（比如 "夹爪卡死" 或 "抓取完毕"）
  rosidl_runtime_c__String message;
} custom_msg__srv__ControlDispatch_Response;

// Struct for a sequence of custom_msg__srv__ControlDispatch_Response.
typedef struct custom_msg__srv__ControlDispatch_Response__Sequence
{
  custom_msg__srv__ControlDispatch_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__srv__ControlDispatch_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__SRV__DETAIL__CONTROL_DISPATCH__STRUCT_H_
