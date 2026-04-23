// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_nav_msgs:msg/PoseEuler.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_NAV_MSGS__MSG__DETAIL__POSE_EULER__FUNCTIONS_H_
#define CUSTOM_NAV_MSGS__MSG__DETAIL__POSE_EULER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "custom_nav_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "custom_nav_msgs/msg/detail/pose_euler__struct.h"

/// Initialize msg/PoseEuler message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_nav_msgs__msg__PoseEuler
 * )) before or use
 * custom_nav_msgs__msg__PoseEuler__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
bool
custom_nav_msgs__msg__PoseEuler__init(custom_nav_msgs__msg__PoseEuler * msg);

/// Finalize msg/PoseEuler message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
void
custom_nav_msgs__msg__PoseEuler__fini(custom_nav_msgs__msg__PoseEuler * msg);

/// Create msg/PoseEuler message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_nav_msgs__msg__PoseEuler__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
custom_nav_msgs__msg__PoseEuler *
custom_nav_msgs__msg__PoseEuler__create();

/// Destroy msg/PoseEuler message.
/**
 * It calls
 * custom_nav_msgs__msg__PoseEuler__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
void
custom_nav_msgs__msg__PoseEuler__destroy(custom_nav_msgs__msg__PoseEuler * msg);

/// Check for msg/PoseEuler message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
bool
custom_nav_msgs__msg__PoseEuler__are_equal(const custom_nav_msgs__msg__PoseEuler * lhs, const custom_nav_msgs__msg__PoseEuler * rhs);

/// Copy a msg/PoseEuler message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
bool
custom_nav_msgs__msg__PoseEuler__copy(
  const custom_nav_msgs__msg__PoseEuler * input,
  custom_nav_msgs__msg__PoseEuler * output);

/// Initialize array of msg/PoseEuler messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_nav_msgs__msg__PoseEuler__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
bool
custom_nav_msgs__msg__PoseEuler__Sequence__init(custom_nav_msgs__msg__PoseEuler__Sequence * array, size_t size);

/// Finalize array of msg/PoseEuler messages.
/**
 * It calls
 * custom_nav_msgs__msg__PoseEuler__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
void
custom_nav_msgs__msg__PoseEuler__Sequence__fini(custom_nav_msgs__msg__PoseEuler__Sequence * array);

/// Create array of msg/PoseEuler messages.
/**
 * It allocates the memory for the array and calls
 * custom_nav_msgs__msg__PoseEuler__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
custom_nav_msgs__msg__PoseEuler__Sequence *
custom_nav_msgs__msg__PoseEuler__Sequence__create(size_t size);

/// Destroy array of msg/PoseEuler messages.
/**
 * It calls
 * custom_nav_msgs__msg__PoseEuler__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
void
custom_nav_msgs__msg__PoseEuler__Sequence__destroy(custom_nav_msgs__msg__PoseEuler__Sequence * array);

/// Check for msg/PoseEuler message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
bool
custom_nav_msgs__msg__PoseEuler__Sequence__are_equal(const custom_nav_msgs__msg__PoseEuler__Sequence * lhs, const custom_nav_msgs__msg__PoseEuler__Sequence * rhs);

/// Copy an array of msg/PoseEuler messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_nav_msgs
bool
custom_nav_msgs__msg__PoseEuler__Sequence__copy(
  const custom_nav_msgs__msg__PoseEuler__Sequence * input,
  custom_nav_msgs__msg__PoseEuler__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_NAV_MSGS__MSG__DETAIL__POSE_EULER__FUNCTIONS_H_
