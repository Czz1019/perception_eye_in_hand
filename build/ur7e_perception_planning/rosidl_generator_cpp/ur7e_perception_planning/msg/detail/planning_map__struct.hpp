// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#ifndef UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__STRUCT_HPP_
#define UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'origin'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur7e_perception_planning__msg__PlanningMap __attribute__((deprecated))
#else
# define DEPRECATED__ur7e_perception_planning__msg__PlanningMap __declspec(deprecated)
#endif

namespace ur7e_perception_planning
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlanningMap_
{
  using Type = PlanningMap_<ContainerAllocator>;

  explicit PlanningMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    origin(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->epoch = 0ull;
      this->sequence = 0ull;
      this->resolution = 0.0f;
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->dimensions.begin(), this->dimensions.end(), 0ul);
      this->unknown_distance = 0.0f;
    }
  }

  explicit PlanningMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    origin(_alloc, _init),
    dimensions(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->epoch = 0ull;
      this->sequence = 0ull;
      this->resolution = 0.0f;
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->dimensions.begin(), this->dimensions.end(), 0ul);
      this->unknown_distance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _epoch_type =
    uint64_t;
  _epoch_type epoch;
  using _sequence_type =
    uint64_t;
  _sequence_type sequence;
  using _origin_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _origin_type origin;
  using _resolution_type =
    float;
  _resolution_type resolution;
  using _dimensions_type =
    std::array<uint32_t, 3>;
  _dimensions_type dimensions;
  using _unknown_distance_type =
    float;
  _unknown_distance_type unknown_distance;
  using _nvblox_distance_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _nvblox_distance_type nvblox_distance;
  using _planning_distance_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _planning_distance_type planning_distance;
  using _free_source_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _free_source_type free_source;
  using _compressed_data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _compressed_data_type compressed_data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__epoch(
    const uint64_t & _arg)
  {
    this->epoch = _arg;
    return *this;
  }
  Type & set__sequence(
    const uint64_t & _arg)
  {
    this->sequence = _arg;
    return *this;
  }
  Type & set__origin(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }
  Type & set__resolution(
    const float & _arg)
  {
    this->resolution = _arg;
    return *this;
  }
  Type & set__dimensions(
    const std::array<uint32_t, 3> & _arg)
  {
    this->dimensions = _arg;
    return *this;
  }
  Type & set__unknown_distance(
    const float & _arg)
  {
    this->unknown_distance = _arg;
    return *this;
  }
  Type & set__nvblox_distance(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->nvblox_distance = _arg;
    return *this;
  }
  Type & set__planning_distance(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->planning_distance = _arg;
    return *this;
  }
  Type & set__free_source(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->free_source = _arg;
    return *this;
  }
  Type & set__compressed_data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->compressed_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur7e_perception_planning__msg__PlanningMap
    std::shared_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur7e_perception_planning__msg__PlanningMap
    std::shared_ptr<ur7e_perception_planning::msg::PlanningMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanningMap_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->epoch != other.epoch) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    if (this->dimensions != other.dimensions) {
      return false;
    }
    if (this->unknown_distance != other.unknown_distance) {
      return false;
    }
    if (this->nvblox_distance != other.nvblox_distance) {
      return false;
    }
    if (this->planning_distance != other.planning_distance) {
      return false;
    }
    if (this->free_source != other.free_source) {
      return false;
    }
    if (this->compressed_data != other.compressed_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanningMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanningMap_

// alias to use template instance with default allocator
using PlanningMap =
  ur7e_perception_planning::msg::PlanningMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ur7e_perception_planning

#endif  // UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__STRUCT_HPP_
