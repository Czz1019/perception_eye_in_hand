// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#ifndef UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__STRUCT_H_
#define UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'origin'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'nvblox_distance'
// Member 'planning_distance'
// Member 'free_source'
// Member 'compressed_data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PlanningMap in the package ur7e_perception_planning.
/**
  * One atomic snapshot. All distances in metres, positive in free space.
  * Arrays use C order [z,y,x]; dimensions are [nx,ny,nz]. Unknown is NEVER free.
 */
typedef struct ur7e_perception_planning__msg__PlanningMap
{
  std_msgs__msg__Header header;
  uint64_t epoch;
  uint64_t sequence;
  geometry_msgs__msg__Point origin;
  float resolution;
  uint32_t dimensions[3];
  float unknown_distance;
  rosidl_runtime_c__float__Sequence nvblox_distance;
  /// Conservative distance to either an obstacle OR unobserved space.
  rosidl_runtime_c__float__Sequence planning_distance;
  /// 0 unknown/occupied, 1 depth-observed free, 2 commissioned initial clearance.
  rosidl_runtime_c__uint8__Sequence free_source;
  /// If nonempty, the three arrays above are empty. Lossless zlib level-1 payload:
  /// n little-endian float32 native distances, n float32 planning distances, n uint8
  /// sources, where n=nx*ny*nz. Geometry/version/stamp remain explicit ROS fields.
  rosidl_runtime_c__uint8__Sequence compressed_data;
} ur7e_perception_planning__msg__PlanningMap;

// Struct for a sequence of ur7e_perception_planning__msg__PlanningMap.
typedef struct ur7e_perception_planning__msg__PlanningMap__Sequence
{
  ur7e_perception_planning__msg__PlanningMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur7e_perception_planning__msg__PlanningMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__STRUCT_H_
