// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_msg:srv/ControlDispatch.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__SRV__DETAIL__CONTROL_DISPATCH__FUNCTIONS_H_
#define CUSTOM_MSG__SRV__DETAIL__CONTROL_DISPATCH__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "custom_msg/msg/rosidl_generator_c__visibility_control.h"

#include "custom_msg/srv/detail/control_dispatch__struct.h"

/// Initialize srv/ControlDispatch message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_msg__srv__ControlDispatch_Request
 * )) before or use
 * custom_msg__srv__ControlDispatch_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Request__init(custom_msg__srv__ControlDispatch_Request * msg);

/// Finalize srv/ControlDispatch message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Request__fini(custom_msg__srv__ControlDispatch_Request * msg);

/// Create srv/ControlDispatch message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_msg__srv__ControlDispatch_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
custom_msg__srv__ControlDispatch_Request *
custom_msg__srv__ControlDispatch_Request__create();

/// Destroy srv/ControlDispatch message.
/**
 * It calls
 * custom_msg__srv__ControlDispatch_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Request__destroy(custom_msg__srv__ControlDispatch_Request * msg);

/// Check for srv/ControlDispatch message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Request__are_equal(const custom_msg__srv__ControlDispatch_Request * lhs, const custom_msg__srv__ControlDispatch_Request * rhs);

/// Copy a srv/ControlDispatch message.
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
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Request__copy(
  const custom_msg__srv__ControlDispatch_Request * input,
  custom_msg__srv__ControlDispatch_Request * output);

/// Initialize array of srv/ControlDispatch messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_msg__srv__ControlDispatch_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Request__Sequence__init(custom_msg__srv__ControlDispatch_Request__Sequence * array, size_t size);

/// Finalize array of srv/ControlDispatch messages.
/**
 * It calls
 * custom_msg__srv__ControlDispatch_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Request__Sequence__fini(custom_msg__srv__ControlDispatch_Request__Sequence * array);

/// Create array of srv/ControlDispatch messages.
/**
 * It allocates the memory for the array and calls
 * custom_msg__srv__ControlDispatch_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
custom_msg__srv__ControlDispatch_Request__Sequence *
custom_msg__srv__ControlDispatch_Request__Sequence__create(size_t size);

/// Destroy array of srv/ControlDispatch messages.
/**
 * It calls
 * custom_msg__srv__ControlDispatch_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Request__Sequence__destroy(custom_msg__srv__ControlDispatch_Request__Sequence * array);

/// Check for srv/ControlDispatch message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Request__Sequence__are_equal(const custom_msg__srv__ControlDispatch_Request__Sequence * lhs, const custom_msg__srv__ControlDispatch_Request__Sequence * rhs);

/// Copy an array of srv/ControlDispatch messages.
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
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Request__Sequence__copy(
  const custom_msg__srv__ControlDispatch_Request__Sequence * input,
  custom_msg__srv__ControlDispatch_Request__Sequence * output);

/// Initialize srv/ControlDispatch message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_msg__srv__ControlDispatch_Response
 * )) before or use
 * custom_msg__srv__ControlDispatch_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Response__init(custom_msg__srv__ControlDispatch_Response * msg);

/// Finalize srv/ControlDispatch message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Response__fini(custom_msg__srv__ControlDispatch_Response * msg);

/// Create srv/ControlDispatch message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_msg__srv__ControlDispatch_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
custom_msg__srv__ControlDispatch_Response *
custom_msg__srv__ControlDispatch_Response__create();

/// Destroy srv/ControlDispatch message.
/**
 * It calls
 * custom_msg__srv__ControlDispatch_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Response__destroy(custom_msg__srv__ControlDispatch_Response * msg);

/// Check for srv/ControlDispatch message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Response__are_equal(const custom_msg__srv__ControlDispatch_Response * lhs, const custom_msg__srv__ControlDispatch_Response * rhs);

/// Copy a srv/ControlDispatch message.
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
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Response__copy(
  const custom_msg__srv__ControlDispatch_Response * input,
  custom_msg__srv__ControlDispatch_Response * output);

/// Initialize array of srv/ControlDispatch messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_msg__srv__ControlDispatch_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Response__Sequence__init(custom_msg__srv__ControlDispatch_Response__Sequence * array, size_t size);

/// Finalize array of srv/ControlDispatch messages.
/**
 * It calls
 * custom_msg__srv__ControlDispatch_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Response__Sequence__fini(custom_msg__srv__ControlDispatch_Response__Sequence * array);

/// Create array of srv/ControlDispatch messages.
/**
 * It allocates the memory for the array and calls
 * custom_msg__srv__ControlDispatch_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
custom_msg__srv__ControlDispatch_Response__Sequence *
custom_msg__srv__ControlDispatch_Response__Sequence__create(size_t size);

/// Destroy array of srv/ControlDispatch messages.
/**
 * It calls
 * custom_msg__srv__ControlDispatch_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
void
custom_msg__srv__ControlDispatch_Response__Sequence__destroy(custom_msg__srv__ControlDispatch_Response__Sequence * array);

/// Check for srv/ControlDispatch message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Response__Sequence__are_equal(const custom_msg__srv__ControlDispatch_Response__Sequence * lhs, const custom_msg__srv__ControlDispatch_Response__Sequence * rhs);

/// Copy an array of srv/ControlDispatch messages.
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
ROSIDL_GENERATOR_C_PUBLIC_custom_msg
bool
custom_msg__srv__ControlDispatch_Response__Sequence__copy(
  const custom_msg__srv__ControlDispatch_Response__Sequence * input,
  custom_msg__srv__ControlDispatch_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__SRV__DETAIL__CONTROL_DISPATCH__FUNCTIONS_H_
