// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice
#include "ur7e_perception_planning/msg/detail/planning_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `origin`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `nvblox_distance`
// Member `planning_distance`
// Member `free_source`
// Member `compressed_data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ur7e_perception_planning__msg__PlanningMap__init(ur7e_perception_planning__msg__PlanningMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
    return false;
  }
  // epoch
  // sequence
  // origin
  if (!geometry_msgs__msg__Point__init(&msg->origin)) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
    return false;
  }
  // resolution
  // dimensions
  // unknown_distance
  // nvblox_distance
  if (!rosidl_runtime_c__float__Sequence__init(&msg->nvblox_distance, 0)) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
    return false;
  }
  // planning_distance
  if (!rosidl_runtime_c__float__Sequence__init(&msg->planning_distance, 0)) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
    return false;
  }
  // free_source
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->free_source, 0)) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
    return false;
  }
  // compressed_data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->compressed_data, 0)) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
    return false;
  }
  return true;
}

void
ur7e_perception_planning__msg__PlanningMap__fini(ur7e_perception_planning__msg__PlanningMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // epoch
  // sequence
  // origin
  geometry_msgs__msg__Point__fini(&msg->origin);
  // resolution
  // dimensions
  // unknown_distance
  // nvblox_distance
  rosidl_runtime_c__float__Sequence__fini(&msg->nvblox_distance);
  // planning_distance
  rosidl_runtime_c__float__Sequence__fini(&msg->planning_distance);
  // free_source
  rosidl_runtime_c__uint8__Sequence__fini(&msg->free_source);
  // compressed_data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->compressed_data);
}

bool
ur7e_perception_planning__msg__PlanningMap__are_equal(const ur7e_perception_planning__msg__PlanningMap * lhs, const ur7e_perception_planning__msg__PlanningMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // epoch
  if (lhs->epoch != rhs->epoch) {
    return false;
  }
  // sequence
  if (lhs->sequence != rhs->sequence) {
    return false;
  }
  // origin
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->origin), &(rhs->origin)))
  {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // dimensions
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->dimensions[i] != rhs->dimensions[i]) {
      return false;
    }
  }
  // unknown_distance
  if (lhs->unknown_distance != rhs->unknown_distance) {
    return false;
  }
  // nvblox_distance
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->nvblox_distance), &(rhs->nvblox_distance)))
  {
    return false;
  }
  // planning_distance
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->planning_distance), &(rhs->planning_distance)))
  {
    return false;
  }
  // free_source
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->free_source), &(rhs->free_source)))
  {
    return false;
  }
  // compressed_data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->compressed_data), &(rhs->compressed_data)))
  {
    return false;
  }
  return true;
}

bool
ur7e_perception_planning__msg__PlanningMap__copy(
  const ur7e_perception_planning__msg__PlanningMap * input,
  ur7e_perception_planning__msg__PlanningMap * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // epoch
  output->epoch = input->epoch;
  // sequence
  output->sequence = input->sequence;
  // origin
  if (!geometry_msgs__msg__Point__copy(
      &(input->origin), &(output->origin)))
  {
    return false;
  }
  // resolution
  output->resolution = input->resolution;
  // dimensions
  for (size_t i = 0; i < 3; ++i) {
    output->dimensions[i] = input->dimensions[i];
  }
  // unknown_distance
  output->unknown_distance = input->unknown_distance;
  // nvblox_distance
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->nvblox_distance), &(output->nvblox_distance)))
  {
    return false;
  }
  // planning_distance
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->planning_distance), &(output->planning_distance)))
  {
    return false;
  }
  // free_source
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->free_source), &(output->free_source)))
  {
    return false;
  }
  // compressed_data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->compressed_data), &(output->compressed_data)))
  {
    return false;
  }
  return true;
}

ur7e_perception_planning__msg__PlanningMap *
ur7e_perception_planning__msg__PlanningMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur7e_perception_planning__msg__PlanningMap * msg = (ur7e_perception_planning__msg__PlanningMap *)allocator.allocate(sizeof(ur7e_perception_planning__msg__PlanningMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur7e_perception_planning__msg__PlanningMap));
  bool success = ur7e_perception_planning__msg__PlanningMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur7e_perception_planning__msg__PlanningMap__destroy(ur7e_perception_planning__msg__PlanningMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur7e_perception_planning__msg__PlanningMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur7e_perception_planning__msg__PlanningMap__Sequence__init(ur7e_perception_planning__msg__PlanningMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur7e_perception_planning__msg__PlanningMap * data = NULL;

  if (size) {
    data = (ur7e_perception_planning__msg__PlanningMap *)allocator.zero_allocate(size, sizeof(ur7e_perception_planning__msg__PlanningMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur7e_perception_planning__msg__PlanningMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur7e_perception_planning__msg__PlanningMap__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ur7e_perception_planning__msg__PlanningMap__Sequence__fini(ur7e_perception_planning__msg__PlanningMap__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ur7e_perception_planning__msg__PlanningMap__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ur7e_perception_planning__msg__PlanningMap__Sequence *
ur7e_perception_planning__msg__PlanningMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur7e_perception_planning__msg__PlanningMap__Sequence * array = (ur7e_perception_planning__msg__PlanningMap__Sequence *)allocator.allocate(sizeof(ur7e_perception_planning__msg__PlanningMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur7e_perception_planning__msg__PlanningMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur7e_perception_planning__msg__PlanningMap__Sequence__destroy(ur7e_perception_planning__msg__PlanningMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur7e_perception_planning__msg__PlanningMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur7e_perception_planning__msg__PlanningMap__Sequence__are_equal(const ur7e_perception_planning__msg__PlanningMap__Sequence * lhs, const ur7e_perception_planning__msg__PlanningMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur7e_perception_planning__msg__PlanningMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur7e_perception_planning__msg__PlanningMap__Sequence__copy(
  const ur7e_perception_planning__msg__PlanningMap__Sequence * input,
  ur7e_perception_planning__msg__PlanningMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur7e_perception_planning__msg__PlanningMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur7e_perception_planning__msg__PlanningMap * data =
      (ur7e_perception_planning__msg__PlanningMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur7e_perception_planning__msg__PlanningMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur7e_perception_planning__msg__PlanningMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur7e_perception_planning__msg__PlanningMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
