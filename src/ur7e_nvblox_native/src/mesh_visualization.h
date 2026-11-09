// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <nvblox_msgs/msg/mesh.hpp>
#include <nvblox/core/types.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

inline bool meshBlockInRoi(const nvblox::Index3D &index, float block_size,
    const nvblox::Vector3f &center, const nvblox::Vector3f &size) {
  auto lo = (center - size / 2.f).eval(), hi = (center + size / 2.f).eval();
  auto a = (index.cast<float>() * block_size).eval();
  auto b = (a + nvblox::Vector3f::Constant(block_size)).eval();
  return !((b.array() <= lo.array()).any() || (a.array() >= hi.array()).any());
}
inline visualization_msgs::msg::Marker clearMeshMarker(const std_msgs::msg::Header &header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

// The official Mesh contains GLOBAL vertex coordinates and local triangle
// indices. A block origin must not be added again. Empty blocks mean DELETE.
inline visualization_msgs::msg::MarkerArray meshMarkersFromMessage(
    const nvblox_msgs::msg::Mesh &mesh, const nvblox::Vector3f &center,
    const nvblox::Vector3f &size) {
  if (mesh.block_indices.size() != mesh.blocks.size() ||
      !std::isfinite(mesh.block_size_m) || mesh.block_size_m <= 0)
    throw std::runtime_error("Invalid native Mesh block layout");
  visualization_msgs::msg::MarkerArray out;
  if (mesh.clear) out.markers.push_back(clearMeshMarker(mesh.header));
  for (size_t i = 0; i < mesh.blocks.size(); ++i) {
    const auto &idx = mesh.block_indices[i];
    const auto &block = mesh.blocks[i];
    if (!meshBlockInRoi({idx.x, idx.y, idx.z}, mesh.block_size_m, center, size)) continue;
    visualization_msgs::msg::Marker marker;
    marker.header = mesh.header;
    marker.ns = std::to_string(idx.x) + "_" + std::to_string(idx.y) + "_" + std::to_string(idx.z);
    marker.id = 0;
    marker.pose.orientation.w = 1.;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = block.triangles.empty() ? visualization_msgs::msg::Marker::DELETE
                                          : visualization_msgs::msg::Marker::ADD;
    marker.color.r = .65f; marker.color.g = .75f; marker.color.b = .85f; marker.color.a = 1.f;
    if (block.triangles.size() % 3) throw std::runtime_error("Mesh triangle list not divisible by 3");
    if (!block.colors.empty() && block.colors.size() != block.vertices.size())
      throw std::runtime_error("Mesh color/vertex count mismatch");
    for (int32_t index : block.triangles) {
      if (index < 0 || size_t(index) >= block.vertices.size())
        throw std::runtime_error("Mesh triangle index outside vertex array");
      const auto &p = block.vertices[index];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        throw std::runtime_error("Non-finite Mesh vertex");
      geometry_msgs::msg::Point q; q.x = p.x; q.y = p.y; q.z = p.z;
      marker.points.push_back(q);
      if (!block.colors.empty()) {
        auto color = block.colors[index]; color.a = 1.f;
        marker.colors.push_back(color);
      }
    }
    out.markers.push_back(std::move(marker));
  }
  return out;
}
