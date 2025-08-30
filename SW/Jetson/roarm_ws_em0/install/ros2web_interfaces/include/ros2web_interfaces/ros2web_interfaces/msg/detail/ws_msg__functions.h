// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ros2web_interfaces:msg/WSMsg.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__FUNCTIONS_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ros2web_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "ros2web_interfaces/msg/detail/ws_msg__struct.h"

/// Initialize msg/WSMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ros2web_interfaces__msg__WSMsg
 * )) before or use
 * ros2web_interfaces__msg__WSMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
bool
ros2web_interfaces__msg__WSMsg__init(ros2web_interfaces__msg__WSMsg * msg);

/// Finalize msg/WSMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
void
ros2web_interfaces__msg__WSMsg__fini(ros2web_interfaces__msg__WSMsg * msg);

/// Create msg/WSMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ros2web_interfaces__msg__WSMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
ros2web_interfaces__msg__WSMsg *
ros2web_interfaces__msg__WSMsg__create();

/// Destroy msg/WSMsg message.
/**
 * It calls
 * ros2web_interfaces__msg__WSMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
void
ros2web_interfaces__msg__WSMsg__destroy(ros2web_interfaces__msg__WSMsg * msg);

/// Check for msg/WSMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
bool
ros2web_interfaces__msg__WSMsg__are_equal(const ros2web_interfaces__msg__WSMsg * lhs, const ros2web_interfaces__msg__WSMsg * rhs);

/// Copy a msg/WSMsg message.
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
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
bool
ros2web_interfaces__msg__WSMsg__copy(
  const ros2web_interfaces__msg__WSMsg * input,
  ros2web_interfaces__msg__WSMsg * output);

/// Initialize array of msg/WSMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * ros2web_interfaces__msg__WSMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
bool
ros2web_interfaces__msg__WSMsg__Sequence__init(ros2web_interfaces__msg__WSMsg__Sequence * array, size_t size);

/// Finalize array of msg/WSMsg messages.
/**
 * It calls
 * ros2web_interfaces__msg__WSMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
void
ros2web_interfaces__msg__WSMsg__Sequence__fini(ros2web_interfaces__msg__WSMsg__Sequence * array);

/// Create array of msg/WSMsg messages.
/**
 * It allocates the memory for the array and calls
 * ros2web_interfaces__msg__WSMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
ros2web_interfaces__msg__WSMsg__Sequence *
ros2web_interfaces__msg__WSMsg__Sequence__create(size_t size);

/// Destroy array of msg/WSMsg messages.
/**
 * It calls
 * ros2web_interfaces__msg__WSMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
void
ros2web_interfaces__msg__WSMsg__Sequence__destroy(ros2web_interfaces__msg__WSMsg__Sequence * array);

/// Check for msg/WSMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
bool
ros2web_interfaces__msg__WSMsg__Sequence__are_equal(const ros2web_interfaces__msg__WSMsg__Sequence * lhs, const ros2web_interfaces__msg__WSMsg__Sequence * rhs);

/// Copy an array of msg/WSMsg messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ros2web_interfaces
bool
ros2web_interfaces__msg__WSMsg__Sequence__copy(
  const ros2web_interfaces__msg__WSMsg__Sequence * input,
  ros2web_interfaces__msg__WSMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__WS_MSG__FUNCTIONS_H_
