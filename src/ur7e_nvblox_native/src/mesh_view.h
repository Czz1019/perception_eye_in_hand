// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mesh_visualization.h"
#include <nvblox/mapper/mapper.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <map>
#include <set>

// Like the reference LayerPublisher, this module owns visualization state.
// All calls and callbacks use the mapper processing group. It never updates
// TSDF/ESDF/Mesh geometry; the official LayerPublisher owns mesh integration.
class MeshView {
public:
  MeshView(rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr group,
           const nvblox::Vector3f &center, const nvblox::Vector3f &size,
           const std::string &frame);
  void updateDemand();
  void afterLayers(nvblox::Mapper &mapper, const nvblox::CudaStream &stream);
  void reset(uint64_t epoch);
  const std::map<std::string, uint64_t> &counts() const { return counts_; }
  double lastSnapshotMs() const { return last_snapshot_ms_; }
private:
  void snapshot(nvblox::Mapper &mapper, const nvblox::CudaStream &stream);
  void clearDisplay();
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  nvblox::Vector3f center_, size_;
  std::string frame_;
  uint64_t epoch_ = 0;
  bool snapshot_pending_ = false;
  std::set<std::string> subscriber_gids_;
  std::map<std::string, uint64_t> counts_;
  double last_snapshot_ms_ = 0;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<nvblox_msgs::msg::Mesh>::SharedPtr mesh_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr republish_;
};
