// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ur7e_perception_planning:msg/PlanningMap.idl
// generated code does not contain a copyright notice

#ifndef UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__TRAITS_HPP_
#define UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ur7e_perception_planning/msg/detail/planning_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'origin'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace ur7e_perception_planning
{

namespace msg
{

inline void to_flow_style_yaml(
  const PlanningMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: epoch
  {
    out << "epoch: ";
    rosidl_generator_traits::value_to_yaml(msg.epoch, out);
    out << ", ";
  }

  // member: sequence
  {
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
    out << ", ";
  }

  // member: origin
  {
    out << "origin: ";
    to_flow_style_yaml(msg.origin, out);
    out << ", ";
  }

  // member: resolution
  {
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << ", ";
  }

  // member: dimensions
  {
    if (msg.dimensions.size() == 0) {
      out << "dimensions: []";
    } else {
      out << "dimensions: [";
      size_t pending_items = msg.dimensions.size();
      for (auto item : msg.dimensions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: unknown_distance
  {
    out << "unknown_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.unknown_distance, out);
    out << ", ";
  }

  // member: nvblox_distance
  {
    if (msg.nvblox_distance.size() == 0) {
      out << "nvblox_distance: []";
    } else {
      out << "nvblox_distance: [";
      size_t pending_items = msg.nvblox_distance.size();
      for (auto item : msg.nvblox_distance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: planning_distance
  {
    if (msg.planning_distance.size() == 0) {
      out << "planning_distance: []";
    } else {
      out << "planning_distance: [";
      size_t pending_items = msg.planning_distance.size();
      for (auto item : msg.planning_distance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: free_source
  {
    if (msg.free_source.size() == 0) {
      out << "free_source: []";
    } else {
      out << "free_source: [";
      size_t pending_items = msg.free_source.size();
      for (auto item : msg.free_source) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: compressed_data
  {
    if (msg.compressed_data.size() == 0) {
      out << "compressed_data: []";
    } else {
      out << "compressed_data: [";
      size_t pending_items = msg.compressed_data.size();
      for (auto item : msg.compressed_data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanningMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: epoch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "epoch: ";
    rosidl_generator_traits::value_to_yaml(msg.epoch, out);
    out << "\n";
  }

  // member: sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
    out << "\n";
  }

  // member: origin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin:\n";
    to_block_style_yaml(msg.origin, out, indentation + 2);
  }

  // member: resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << "\n";
  }

  // member: dimensions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dimensions.size() == 0) {
      out << "dimensions: []\n";
    } else {
      out << "dimensions:\n";
      for (auto item : msg.dimensions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: unknown_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unknown_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.unknown_distance, out);
    out << "\n";
  }

  // member: nvblox_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nvblox_distance.size() == 0) {
      out << "nvblox_distance: []\n";
    } else {
      out << "nvblox_distance:\n";
      for (auto item : msg.nvblox_distance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: planning_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.planning_distance.size() == 0) {
      out << "planning_distance: []\n";
    } else {
      out << "planning_distance:\n";
      for (auto item : msg.planning_distance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: free_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.free_source.size() == 0) {
      out << "free_source: []\n";
    } else {
      out << "free_source:\n";
      for (auto item : msg.free_source) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: compressed_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.compressed_data.size() == 0) {
      out << "compressed_data: []\n";
    } else {
      out << "compressed_data:\n";
      for (auto item : msg.compressed_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanningMap & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ur7e_perception_planning

namespace rosidl_generator_traits
{

[[deprecated("use ur7e_perception_planning::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur7e_perception_planning::msg::PlanningMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur7e_perception_planning::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur7e_perception_planning::msg::to_yaml() instead")]]
inline std::string to_yaml(const ur7e_perception_planning::msg::PlanningMap & msg)
{
  return ur7e_perception_planning::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ur7e_perception_planning::msg::PlanningMap>()
{
  return "ur7e_perception_planning::msg::PlanningMap";
}

template<>
inline const char * name<ur7e_perception_planning::msg::PlanningMap>()
{
  return "ur7e_perception_planning/msg/PlanningMap";
}

template<>
struct has_fixed_size<ur7e_perception_planning::msg::PlanningMap>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ur7e_perception_planning::msg::PlanningMap>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ur7e_perception_planning::msg::PlanningMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UR7E_PERCEPTION_PLANNING__MSG__DETAIL__PLANNING_MAP__TRAITS_HPP_
