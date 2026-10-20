// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#ifndef UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ur7e_perception_planning/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ur7e_perception_planning/msg/detail/planning_map__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ur7e_perception_planning
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ur7e_perception_planning
cdr_serialize(
  const ur7e_perception_planning::msg::PlanningMap & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ur7e_perception_planning
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ur7e_perception_planning::msg::PlanningMap & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ur7e_perception_planning
get_serialized_size(
  const ur7e_perception_planning::msg::PlanningMap & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ur7e_perception_planning
max_serialized_size_PlanningMap(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ur7e_perception_planning

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ur7e_perception_planning
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ur7e_perception_planning, msg, PlanningMap)();

#ifdef __cplusplus
}
#endif

#endif  // UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
