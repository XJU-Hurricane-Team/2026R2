// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/PoseEuler.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__POSE_EULER__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__POSE_EULER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PoseEuler in the package custom_msg.
typedef struct custom_msg__msg__PoseEuler
{
  float x;
  float y;
  float z;
  float yaw;
} custom_msg__msg__PoseEuler;

// Struct for a sequence of custom_msg__msg__PoseEuler.
typedef struct custom_msg__msg__PoseEuler__Sequence
{
  custom_msg__msg__PoseEuler * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__PoseEuler__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__POSE_EULER__STRUCT_H_
