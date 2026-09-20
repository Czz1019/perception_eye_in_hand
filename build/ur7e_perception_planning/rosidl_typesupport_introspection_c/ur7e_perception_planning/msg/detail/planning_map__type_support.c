// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ur7e_perception_planning/msg/detail/planning_map__rosidl_typesupport_introspection_c.h"
#include "ur7e_perception_planning/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ur7e_perception_planning/msg/detail/planning_map__functions.h"
#include "ur7e_perception_planning/msg/detail/planning_map__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `origin`
#include "geometry_msgs/msg/point.h"
// Member `origin`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `nvblox_distance`
// Member `planning_distance`
// Member `free_source`
// Member `compressed_data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ur7e_perception_planning__msg__PlanningMap__init(message_memory);
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_fini_function(void * message_memory)
{
  ur7e_perception_planning__msg__PlanningMap__fini(message_memory);
}

size_t ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__dimensions(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__dimensions(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__dimensions(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__dimensions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__dimensions(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__dimensions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__dimensions(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

size_t ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__nvblox_distance(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__nvblox_distance(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__nvblox_distance(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__nvblox_distance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__nvblox_distance(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__nvblox_distance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__nvblox_distance(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__nvblox_distance(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__planning_distance(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__planning_distance(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__planning_distance(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__planning_distance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__planning_distance(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__planning_distance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__planning_distance(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__planning_distance(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__free_source(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__free_source(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__free_source(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__free_source(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__free_source(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__free_source(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__free_source(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__free_source(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__compressed_data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__compressed_data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__compressed_data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__compressed_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__compressed_data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__compressed_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__compressed_data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__compressed_data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_member_array[11] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "epoch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, epoch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sequence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, sequence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "origin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, origin),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resolution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, resolution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dimensions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, dimensions),  // bytes offset in struct
    NULL,  // default value
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__dimensions,  // size() function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__dimensions,  // get_const(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__dimensions,  // get(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__dimensions,  // fetch(index, &value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__dimensions,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "unknown_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, unknown_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nvblox_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, nvblox_distance),  // bytes offset in struct
    NULL,  // default value
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__nvblox_distance,  // size() function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__nvblox_distance,  // get_const(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__nvblox_distance,  // get(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__nvblox_distance,  // fetch(index, &value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__nvblox_distance,  // assign(index, value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__nvblox_distance  // resize(index) function pointer
  },
  {
    "planning_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, planning_distance),  // bytes offset in struct
    NULL,  // default value
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__planning_distance,  // size() function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__planning_distance,  // get_const(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__planning_distance,  // get(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__planning_distance,  // fetch(index, &value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__planning_distance,  // assign(index, value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__planning_distance  // resize(index) function pointer
  },
  {
    "free_source",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, free_source),  // bytes offset in struct
    NULL,  // default value
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__free_source,  // size() function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__free_source,  // get_const(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__free_source,  // get(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__free_source,  // fetch(index, &value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__free_source,  // assign(index, value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__free_source  // resize(index) function pointer
  },
  {
    "compressed_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur7e_perception_planning__msg__PlanningMap, compressed_data),  // bytes offset in struct
    NULL,  // default value
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__size_function__PlanningMap__compressed_data,  // size() function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_const_function__PlanningMap__compressed_data,  // get_const(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__get_function__PlanningMap__compressed_data,  // get(index) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__fetch_function__PlanningMap__compressed_data,  // fetch(index, &value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__assign_function__PlanningMap__compressed_data,  // assign(index, value) function pointer
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__resize_function__PlanningMap__compressed_data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_members = {
  "ur7e_perception_planning__msg",  // message namespace
  "PlanningMap",  // message name
  11,  // number of fields
  sizeof(ur7e_perception_planning__msg__PlanningMap),
  ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_member_array,  // message members
  ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_init_function,  // function to initialize message memory (memory has to be allocated)
  ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_type_support_handle = {
  0,
  &ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur7e_perception_planning
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur7e_perception_planning, msg, PlanningMap)() {
  ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_type_support_handle.typesupport_identifier) {
    ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ur7e_perception_planning__msg__PlanningMap__rosidl_typesupport_introspection_c__PlanningMap_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
