# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ur7e_perception_planning_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ur7e_perception_planning_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ur7e_perception_planning_FOUND FALSE)
  elseif(NOT ur7e_perception_planning_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ur7e_perception_planning_FOUND FALSE)
  endif()
  return()
endif()
set(_ur7e_perception_planning_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ur7e_perception_planning_FIND_QUIETLY)
  message(STATUS "Found ur7e_perception_planning: 0.1.0 (${ur7e_perception_planning_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ur7e_perception_planning' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ur7e_perception_planning_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ur7e_perception_planning_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "rosidl_cmake-extras.cmake;ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_targets-extras.cmake;rosidl_cmake_export_typesupport_targets-extras.cmake;rosidl_cmake_export_typesupport_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${ur7e_perception_planning_DIR}/${_extra}")
endforeach()
