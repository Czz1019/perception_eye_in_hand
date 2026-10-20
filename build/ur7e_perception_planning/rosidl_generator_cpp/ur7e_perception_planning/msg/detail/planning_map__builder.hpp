// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#ifndef UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__BUILDER_HPP_
#define UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur7e_perception_planning/msg/detail/planning_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur7e_perception_planning
{

namespace msg
{

namespace builder
{

class Init_PlanningMap_compressed_data
{
public:
  explicit Init_PlanningMap_compressed_data(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  ::ur7e_perception_planning::msg::PlanningMap compressed_data(::ur7e_perception_planning::msg::PlanningMap::_compressed_data_type arg)
  {
    msg_.compressed_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_free_source
{
public:
  explicit Init_PlanningMap_free_source(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_compressed_data free_source(::ur7e_perception_planning::msg::PlanningMap::_free_source_type arg)
  {
    msg_.free_source = std::move(arg);
    return Init_PlanningMap_compressed_data(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_planning_distance
{
public:
  explicit Init_PlanningMap_planning_distance(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_free_source planning_distance(::ur7e_perception_planning::msg::PlanningMap::_planning_distance_type arg)
  {
    msg_.planning_distance = std::move(arg);
    return Init_PlanningMap_free_source(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_nvblox_distance
{
public:
  explicit Init_PlanningMap_nvblox_distance(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_planning_distance nvblox_distance(::ur7e_perception_planning::msg::PlanningMap::_nvblox_distance_type arg)
  {
    msg_.nvblox_distance = std::move(arg);
    return Init_PlanningMap_planning_distance(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_unknown_distance
{
public:
  explicit Init_PlanningMap_unknown_distance(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_nvblox_distance unknown_distance(::ur7e_perception_planning::msg::PlanningMap::_unknown_distance_type arg)
  {
    msg_.unknown_distance = std::move(arg);
    return Init_PlanningMap_nvblox_distance(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_dimensions
{
public:
  explicit Init_PlanningMap_dimensions(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_unknown_distance dimensions(::ur7e_perception_planning::msg::PlanningMap::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return Init_PlanningMap_unknown_distance(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_resolution
{
public:
  explicit Init_PlanningMap_resolution(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_dimensions resolution(::ur7e_perception_planning::msg::PlanningMap::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return Init_PlanningMap_dimensions(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_origin
{
public:
  explicit Init_PlanningMap_origin(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_resolution origin(::ur7e_perception_planning::msg::PlanningMap::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return Init_PlanningMap_resolution(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_sequence
{
public:
  explicit Init_PlanningMap_sequence(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_origin sequence(::ur7e_perception_planning::msg::PlanningMap::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return Init_PlanningMap_origin(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_epoch
{
public:
  explicit Init_PlanningMap_epoch(::ur7e_perception_planning::msg::PlanningMap & msg)
  : msg_(msg)
  {}
  Init_PlanningMap_sequence epoch(::ur7e_perception_planning::msg::PlanningMap::_epoch_type arg)
  {
    msg_.epoch = std::move(arg);
    return Init_PlanningMap_sequence(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

class Init_PlanningMap_header
{
public:
  Init_PlanningMap_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanningMap_epoch header(::ur7e_perception_planning::msg::PlanningMap::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PlanningMap_epoch(msg_);
  }

private:
  ::ur7e_perception_planning::msg::PlanningMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur7e_perception_planning::msg::PlanningMap>()
{
  return ur7e_perception_planning::msg::builder::Init_PlanningMap_header();
}

}  // namespace ur7e_perception_planning

#endif  // UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__BUILDER_HPP_
