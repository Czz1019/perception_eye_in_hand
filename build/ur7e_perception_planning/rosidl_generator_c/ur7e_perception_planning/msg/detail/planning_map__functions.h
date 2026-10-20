// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#ifndef UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__FUNCTIONS_H_
#define UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ur7e_perception_planning/msg/rosidl_generator_c__visibility_control.h"

#include "ur7e_perception_planning/msg/detail/planning_map__struct.h"

/// Initialize msg/PlanningMap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ur7e_perception_planning__msg__PlanningMap
 * )) before or use
 * ur7e_perception_planning__msg__PlanningMap__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
bool
ur7e_perception_planning__msg__PlanningMap__init(ur7e_perception_planning__msg__PlanningMap * msg);

/// Finalize msg/PlanningMap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
void
ur7e_perception_planning__msg__PlanningMap__fini(ur7e_perception_planning__msg__PlanningMap * msg);

/// Create msg/PlanningMap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ur7e_perception_planning__msg__PlanningMap__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
ur7e_perception_planning__msg__PlanningMap *
ur7e_perception_planning__msg__PlanningMap__create();

/// Destroy msg/PlanningMap message.
/**
 * It calls
 * ur7e_perception_planning__msg__PlanningMap__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
void
ur7e_perception_planning__msg__PlanningMap__destroy(ur7e_perception_planning__msg__PlanningMap * msg);

/// Check for msg/PlanningMap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
bool
ur7e_perception_planning__msg__PlanningMap__are_equal(const ur7e_perception_planning__msg__PlanningMap * lhs, const ur7e_perception_planning__msg__PlanningMap * rhs);

/// Copy a msg/PlanningMap message.
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
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
bool
ur7e_perception_planning__msg__PlanningMap__copy(
  const ur7e_perception_planning__msg__PlanningMap * input,
  ur7e_perception_planning__msg__PlanningMap * output);

/// Initialize array of msg/PlanningMap messages.
/**
 * It allocates the memory for the number of elements and calls
 * ur7e_perception_planning__msg__PlanningMap__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
bool
ur7e_perception_planning__msg__PlanningMap__Sequence__init(ur7e_perception_planning__msg__PlanningMap__Sequence * array, size_t size);

/// Finalize array of msg/PlanningMap messages.
/**
 * It calls
 * ur7e_perception_planning__msg__PlanningMap__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
void
ur7e_perception_planning__msg__PlanningMap__Sequence__fini(ur7e_perception_planning__msg__PlanningMap__Sequence * array);

/// Create array of msg/PlanningMap messages.
/**
 * It allocates the memory for the array and calls
 * ur7e_perception_planning__msg__PlanningMap__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
ur7e_perception_planning__msg__PlanningMap__Sequence *
ur7e_perception_planning__msg__PlanningMap__Sequence__create(size_t size);

/// Destroy array of msg/PlanningMap messages.
/**
 * It calls
 * ur7e_perception_planning__msg__PlanningMap__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
void
ur7e_perception_planning__msg__PlanningMap__Sequence__destroy(ur7e_perception_planning__msg__PlanningMap__Sequence * array);

/// Check for msg/PlanningMap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
bool
ur7e_perception_planning__msg__PlanningMap__Sequence__are_equal(const ur7e_perception_planning__msg__PlanningMap__Sequence * lhs, const ur7e_perception_planning__msg__PlanningMap__Sequence * rhs);

/// Copy an array of msg/PlanningMap messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ur7e_perception_planning
bool
ur7e_perception_planning__msg__PlanningMap__Sequence__copy(
  const ur7e_perception_planning__msg__PlanningMap__Sequence * input,
  ur7e_perception_planning__msg__PlanningMap__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__FUNCTIONS_H_
