// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_nav_msgs:msg/PoseEuler.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_NAV_MSGS__MSG__DETAIL__POSE_EULER__STRUCT_H_
#define CUSTOM_NAV_MSGS__MSG__DETAIL__POSE_EULER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PoseEuler in the package custom_nav_msgs.
typedef struct custom_nav_msgs__msg__PoseEuler
{
  float x;
  float y;
  float z;
  float yaw;
} custom_nav_msgs__msg__PoseEuler;

// Struct for a sequence of custom_nav_msgs__msg__PoseEuler.
typedef struct custom_nav_msgs__msg__PoseEuler__Sequence
{
  custom_nav_msgs__msg__PoseEuler * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_nav_msgs__msg__PoseEuler__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_NAV_MSGS__MSG__DETAIL__POSE_EULER__STRUCT_H_
