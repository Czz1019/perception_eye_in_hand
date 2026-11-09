// SPDX-License-Identifier: Apache-2.0
#include "mesh_view.h"
#include <nvblox_ros/conversions/mesh_conversions.hpp>
#include <chrono>

MeshView::MeshView(rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr group,
                   const nvblox::Vector3f &center, const nvblox::Vector3f &size,
                   const std::string &frame)
    : node_(node), group_(group), center_(center), size_(size), frame_(frame) {
  markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/mesh_markers", rclcpp::QoS(10).reliable());
  republish_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/republish_mesh", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                 std_srvs::srv::Trigger::Response::SharedPtr response) {
        snapshot_pending_ = markers_->get_subscription_count() > 0;
        response->success = snapshot_pending_;
        response->message = snapshot_pending_ ? "Snapshot of official Mesh scheduled" : "Enable mesh_markers first";
      }, rmw_qos_profile_services_default, group_);
}

void MeshView::updateDemand() {
  if (!markers_->get_subscription_count()) {
    mesh_sub_.reset(); subscriber_gids_.clear(); snapshot_pending_ = false;
    return;
  }
  if (mesh_sub_) return;
  snapshot_pending_ = true;
  rclcpp::SubscriptionOptions options; options.callback_group = group_;
  mesh_sub_ = node_->create_subscription<nvblox_msgs::msg::Mesh>(
      "~/mesh", rclcpp::QoS(10).reliable(),
      [this, epoch = epoch_](nvblox_msgs::msg::Mesh::ConstSharedPtr msg) {
        if (epoch != epoch_ || !markers_->get_subscription_count()) return;
        try {
          auto markers = meshMarkersFromMessage(*msg, center_, size_);
          if (msg->clear) snapshot_pending_ = true;
          markers_->publish(markers);
          ++counts_["mesh_marker_updates"];
        } catch (const std::exception &e) {
          ++counts_["invalid_mesh_messages"];
          RCLCPP_ERROR(node_->get_logger(), "Rejected native Mesh: %s", e.what());
        }
      }, options);
}

void MeshView::afterLayers(nvblox::Mapper &mapper, const nvblox::CudaStream &stream) {
  if (!markers_->get_subscription_count()) return;
  std::set<std::string> current;
  for (const auto &endpoint : node_->get_subscriptions_info_by_topic(markers_->get_topic_name())) {
    const auto &gid = endpoint.endpoint_gid();
    std::string id(reinterpret_cast<const char *>(gid.data()), gid.size());
    if (!subscriber_gids_.count(id)) snapshot_pending_ = true;
    current.insert(std::move(id));
  }
  subscriber_gids_ = std::move(current);
  if (snapshot_pending_) snapshot(mapper, stream);
}

void MeshView::snapshot(nvblox::Mapper &mapper, const nvblox::CudaStream &stream) {
  auto start = std::chrono::steady_clock::now();
  // publishLayers() already called the official incremental updateMesh().
  // A viewer reconnect must not trigger updateMesh(kYes) over the whole map.
  const auto &layer = mapper.mesh_layer();
  std::vector<nvblox::Index3D> selected;
  for (const auto &index : layer.getAllBlockIndices())
    if (meshBlockInRoi(index, layer.block_size(), center_, size_)) selected.push_back(index);
  visualization_msgs::msg::MarkerArray out;
  if (!selected.empty()) {
    auto serialized = mapper.layer_streamers().serializeAllBlocks(layer, selected, stream);
    if (!serialized) throw std::runtime_error("Official Mesh serializer unavailable");
    nvblox::conversions::markerMessageFromSerializedMesh(serialized, frame_, &out);
  }
  std_msgs::msg::Header header; header.frame_id = frame_; header.stamp = node_->now();
  for (auto &marker : out.markers) marker.header = header;
  out.markers.insert(out.markers.begin(), clearMeshMarker(header));
  markers_->publish(out);
  snapshot_pending_ = false;
  ++counts_["mesh_marker_snapshots"];
  last_snapshot_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
}

void MeshView::clearDisplay() {
  std_msgs::msg::Header header; header.frame_id = frame_; header.stamp = node_->now();
  visualization_msgs::msg::MarkerArray out;
  out.markers.push_back(clearMeshMarker(header)); markers_->publish(out);
}

void MeshView::reset(uint64_t epoch) {
  epoch_ = epoch; mesh_sub_.reset(); subscriber_gids_.clear();
  snapshot_pending_ = markers_->get_subscription_count() > 0;
  clearDisplay();
}
