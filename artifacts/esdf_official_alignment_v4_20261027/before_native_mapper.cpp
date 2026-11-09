// SPDX-License-Identifier: Apache-2.0
#include "block_export.h"
#include "mesh_visualization.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <glog/logging.h>
#include <map>
#include <set>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <nvblox_ros/nvblox_node.hpp>
#include <nvblox_ros/conversions/mesh_conversions.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>
using Json = nlohmann::json;
using Wall = std::chrono::steady_clock;
static double seconds(Wall::time_point t) {
  return std::chrono::duration<double>(Wall::now() - t).count();
}

class NativeMapper final : public nvblox::NvbloxNode {
public:
  NativeMapper() : NvbloxNode(rclcpp::NodeOptions(), "ur7e_nvblox") {
    if (get_parameter("esdf_mode").as_string() != "3d" ||
        get_parameter("mapping_type").as_string() != "static_tsdf")
      throw std::runtime_error("This adapter requires static_tsdf and 3d ESDF");
    auto c =
        declare_parameter<std::vector<double>>("export_center_m", {0., 0., .5});
    auto s =
        declare_parameter<std::vector<double>>("export_size_m", {2., 2., 1.5});
    if (c.size() != 3 || s.size() != 3)
      throw std::runtime_error("Export geometry needs 3 coordinates");
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(c[i]) || !std::isfinite(s[i]) || s[i] <= 0)
        throw std::runtime_error("Invalid export geometry");
      center_[i] = c[i];
      size_[i] = s[i];
    }
    band_ = declare_parameter("export_band_m", .15);
    export_hz_ = declare_parameter("export_hz", 30.);
    max_age_ = declare_parameter("max_frame_age_sec", .5);
    expected_frame_ = declare_parameter<std::string>(
        "expected_depth_frame", "camera_depth_optical_frame");
    expected_color_frame_ = declare_parameter<std::string>(
        "expected_color_frame", "camera_color_optical_frame");
    if (!std::isfinite(band_) || !std::isfinite(export_hz_) || !std::isfinite(max_age_) ||
        band_ <= 0 || export_hz_ <= 0 || max_age_ <= 0)
      throw std::runtime_error("Positive export limits required");
    path_max_ = declare_parameter<int>("path_max_poses", 2000);
    path_hz_ = declare_parameter("path_publish_hz", 10.);
    path_min_translation_ = declare_parameter("path_min_translation_m", .005);
    path_min_rotation_ = declare_parameter("path_min_rotation_rad", .02);
    if (path_max_ < 2 || !std::isfinite(path_hz_) || path_hz_ <= 0 ||
        !std::isfinite(path_min_translation_) || path_min_translation_ < 0 ||
        !std::isfinite(path_min_rotation_) || path_min_rotation_ < 0)
      throw std::runtime_error("Invalid camera path limits");
    observed_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/esdf_distance_band", rclcpp::SensorDataQoS().keep_last(1));
    occupied_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/esdf_occupied", rclcpp::QoS(1).reliable());
    if (declare_parameter("publish_full_distance_cloud", false))
      distance_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/esdf_distance", rclcpp::SensorDataQoS().keep_last(1));
    pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/camera_pose", rclcpp::SensorDataQoS().keep_last(1));
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    path_ = create_publisher<nav_msgs::msg::Path>("~/camera_path", retained);
    epoch_pub_ = create_publisher<std_msgs::msg::UInt64>("~/map_epoch", retained);
    epoch_pub_->publish(std_msgs::msg::UInt64());
    mesh_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/mesh_markers", rclcpp::QoS(10).reliable());
    stats_ = create_publisher<std_msgs::msg::String>("~/stats", 2);
    topic_info_ = create_publisher<std_msgs::msg::String>("~/topic_info", retained);
    publishTopicInfo();
    stats_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this] { publishStats(); }, group_processing_);
    reset_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_map",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          resetMap();
          response->success = true;
          response->message = "Native map cleared; awaiting a new image";
        },
        rmw_qos_profile_services_default, group_processing_);
    reset_stats_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_stats",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          latencies_.clear();
          response->success = true;
        },
        rmw_qos_profile_services_default, group_processing_);
    republish_mesh_ = create_service<std_srvs::srv::Trigger>(
        "~/republish_mesh",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = mesh_markers_->get_subscription_count() > 0;
          mesh_snapshot_pending_ = response->success;
          response->message = response->success ? "Mesh snapshot scheduled" : "Enable a mesh_markers consumer first";
        }, rmw_qos_profile_services_default, group_processing_);
    RCLCPP_INFO(get_logger(),
                "Native adapter: official ROS input/TF/Mapper/services/Mesh, "
                "allocated-block sparse ESDF (no unknown padding), %.4f m voxels; "
                "camera_path uses robot TF; RGB fusion: %s",
                static_mapper_->voxel_size_m(), params_.use_color.get() ? "enabled" : "disabled");
  }

protected:
  void processDepthQueue() override {
    {
      std::lock_guard<std::mutex> lock(depth_queue_mutex_);
      for (auto &item : *depth_image_queue_) {
        auto [view, info, mask, maskinfo] = decomposeImageTypeVariant(item);
        int64_t ns = rclcpp::Time(view->GetTimestampSeconds(),
                                  view->GetTimestampNanoseconds(), RCL_ROS_TIME)
                         .nanoseconds();
        if (!seen_.count(ns)) {
          seen_[ns] = Wall::now();
          ++counts_["input_queue_seen"];
        }
      }
      while (seen_.size() > 256)
        seen_.erase(seen_.begin());
    }
    NvbloxNode::processDepthQueue();
  }
  bool processDepthImage(const ImageTypeVariant &item) override {
    auto [view, info, mask, maskinfo] = decomposeImageTypeVariant(item);
    rclcpp::Time stamp(view->GetTimestampSeconds(),
                       view->GetTimestampNanoseconds(), RCL_ROS_TIME);
    if (view->GetFrameId() != expected_frame_ ||
        info->header.frame_id != expected_frame_) {
      ++counts_["calibration_mismatch"];
      return true;
    }
    double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || std::abs(age) > max_age_ ||
        stamp <= newest_integrated_depth_time_) {
      ++counts_["stale_or_out_of_order_depth"];
      return true;
    }
    auto before = newest_integrated_depth_time_;
    auto start = Wall::now();
    bool ok = NvbloxNode::processDepthImage(item);
    if (newest_integrated_depth_time_ > before) {
      cuda_stream_->synchronize();
      add("depth_process_complete", seconds(start) * 1000);
      ++counts_["depth_integrated"];
      auto found = seen_.find(stamp.nanoseconds());
      last_input_wall_ = found == seen_.end() ? start : found->second;
      input_width_ = info->width;
      input_height_ = info->height;
      publishCameraPose(T_L_C_depth_, stamp);
    }
    return ok;
  }
  bool processColorImage(const ImageTypeVariant &item) override {
    auto [view, info, mask, maskinfo] = decomposeImageTypeVariant(item);
    ++counts_["color_process_attempts"];
    rclcpp::Time stamp(view->GetTimestampSeconds(),
                       view->GetTimestampNanoseconds(), RCL_ROS_TIME);
    if (view->GetFrameId() != expected_color_frame_ ||
        info->header.frame_id != expected_color_frame_ ||
        info->width != view->GetWidth() || info->height != view->GetHeight() ||
        !std::isfinite(info->k[0]) || !std::isfinite(info->k[4]) ||
        info->k[0] <= 0 || info->k[4] <= 0) {
      ++counts_["color_calibration_mismatch"];
      return true;
    }
    if (stamp.nanoseconds() <= 0 || std::abs((now() - stamp).seconds()) > max_age_) {
      ++counts_["stale_color"];
      return true;
    }
    // The selected official conversion supports rgb8 and bgra8. Reject other
    // encodings explicitly instead of repeatedly retrying an unusable image.
    const auto encoding = view->GetEncoding();
    if (encoding != "rgb8" && encoding != "bgra8") {
      ++counts_["unsupported_color_encoding"];
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "RGB fusion requires rgb8/bgra8; got %s", encoding.c_str());
      return true;
    }
    const auto frame = view->GetFrameId();
    auto previous = integrate_color_last_times_.find(frame);
    const int64_t before = previous == integrate_color_last_times_.end() ? 0 : previous->second.nanoseconds();
    auto start = Wall::now();
    bool ok = NvbloxNode::processColorImage(item);
    auto integrated = integrate_color_last_times_.find(frame);
    if (integrated != integrate_color_last_times_.end() && integrated->second.nanoseconds() > before) {
      ++counts_["color_integrated"];
      add("color_process_submit", seconds(start) * 1000);
      color_stamp_ = stamp;
      color_width_ = info->width; color_height_ = info->height;
      color_encoding_ = encoding;
    } else if (!ok) {
      ++counts_["color_tf_wait_attempts"];
    } else {
      ++counts_["color_rate_or_order_skipped"];
    }
    return ok;
  }
  void processEsdf() override {
    if (newest_integrated_depth_time_.nanoseconds() <= 0 ||
        newest_integrated_depth_time_ <= esdf_stamp_)
      return;
    auto start = Wall::now();
    NvbloxNode::processEsdf();
    cuda_stream_->synchronize();
    add("esdf_complete", seconds(start) * 1000);
    esdf_stamp_ = newest_integrated_depth_time_;
    esdf_pose_ = T_L_C_depth_;
    esdf_input_wall_ = last_input_wall_;
    ++version_;
    ++counts_["esdf_updated"];
    add("queue_seen_to_esdf", seconds(esdf_input_wall_) * 1000);
  }
  void tick() override {
    const auto current = now().nanoseconds();
    if (last_ros_ns_ > 0 && current < last_ros_ns_)
      resetMap();
    last_ros_ns_ = current;
    // Official processing modules, paced by steady time. Gazebo /clock can
    // arrive at only 10 Hz; it remains the image/TF timebase, not a work clock.
    processServiceRequestTaskQueue();
    if (params_.use_depth.get())
      processDepthQueue();
    if (params_.use_color.get())
      processColorQueue();
    if (params_.use_lidar.get())
      processPointcloudQueue();
    double t =
        std::chrono::duration<double>(Wall::now().time_since_epoch()).count();
    if (newest_integrated_depth_time_ > esdf_stamp_) {
      if (budget_last_ > 0)
        budget_ = std::min(2., budget_ + (t - budget_last_) *
                                             params_.update_esdf_rate_hz.get());
      budget_last_ = t;
      if (params_.update_esdf_rate_hz.get() > 0 && budget_ >= 1. - 1e-6) {
        budget_ -= 1.;
        processEsdf();
      }
    }
    auto due = [&](const std::string &key, double hz) {
      if (hz <= 0)
        return false;
      auto &last = scheduled_[key];
      if (t - last < 1. / hz)
        return false;
      last = t;
      return true;
    };
    if (due("decay", params_.decay_tsdf_rate_hz.get()))
      decayTsdf();
    if (due("clear", params_.clear_map_outside_radius_rate_hz.get()))
      clearMapOutsideOfRadiusOfLastKnownPose();
    updateMeshViewDemand();
    if (due("layers", params_.publish_layer_rate_hz.get())) {
      refreshMeshSubscriberIdentities();
      publishLayers();
      if (mesh_snapshot_pending_ && mesh_markers_->get_subscription_count())
        publishMeshSnapshot();
    }
    if (due("debug", params_.publish_debug_vis_rate_hz.get()))
      publishDebugVisualizations();
    if (!version_ || esdf_stamp_.nanoseconds() <= 0 ||
        std::abs((now() - esdf_stamp_).seconds()) > max_age_)
      return;
    bool obs = observed_->get_subscription_count() > 0 &&
               observed_version_ != version_;
    bool occupied = occupied_->get_subscription_count() > 0 && occupied_version_ != version_;
    bool distance = distance_ && distance_->get_subscription_count() > 0 && distance_version_ != version_;
    if (!(obs || occupied || distance) || seconds(last_export_) < 1. / export_hz_)
      return;
    auto start = Wall::now();
    auto output = exporter_.run(static_mapper_->esdf_layer(), center_, size_,
                                band_, {occupied, obs, distance}, *cuda_stream_);
    add("block_export_readback", seconds(start) * 1000);
    last_export_ = start;
    ++counts_["esdf_exported"];
    exported_version_ = version_;
    candidate_count_ = output.visited_voxels;
    allocated_blocks_ = output.allocated_blocks;
    selected_blocks_ = output.selected_blocks;
    point_count_ = output.band.size();
    occupied_count_ = output.occupied.size();
    distance_count_ = output.distance.size();
    if (obs) {
      observed_->publish(cloud(output.band));
      observed_version_ = version_;
      ++counts_["observed_cloud_published"];
    }
    if (occupied) {
      occupied_->publish(cloud(output.occupied));
      occupied_version_ = version_;
      ++counts_["occupied_cloud_published"];
    }
    if (distance) {
      distance_->publish(cloud(output.distance));
      distance_version_ = version_;
      ++counts_["distance_cloud_published"];
    }
    ++counts_["esdf_published"];
    add("queue_seen_to_publish", seconds(esdf_input_wall_) * 1000);
  }

private:
  void publishCameraPose(const nvblox::Transform &pose, const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = params_.global_frame.get();
    msg.header.stamp = stamp;
    const auto t = pose.translation();
    Eigen::Quaternionf q(pose.rotation());
    msg.pose.position.x = t.x(); msg.pose.position.y = t.y(); msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
    pose_->publish(msg);
    bool append = camera_path_.empty();
    if (!append) {
      const auto &last = camera_path_.back();
      nvblox::Vector3f old(last.pose.position.x, last.pose.position.y, last.pose.position.z);
      Eigen::Quaternionf old_q(last.pose.orientation.w, last.pose.orientation.x,
                                last.pose.orientation.y, last.pose.orientation.z);
      append = (t - old).norm() >= path_min_translation_ ||
               q.angularDistance(old_q) >= path_min_rotation_ ||
               (stamp - rclcpp::Time(last.header.stamp)).seconds() >= 1.;
    }
    if (append) {
      camera_path_.push_back(msg);
      while (camera_path_.size() > size_t(path_max_)) camera_path_.pop_front();
      path_dirty_ = true;
    }
    if (path_dirty_ && seconds(last_path_publish_) >= 1. / path_hz_) {
      nav_msgs::msg::Path path;
      path.header = msg.header;
      path.poses.assign(camera_path_.begin(), camera_path_.end());
      path_->publish(path);
      path_dirty_ = false; last_path_publish_ = Wall::now();
      ++counts_["camera_path_published"];
    }
  }

  void updateMeshViewDemand() {
    size_t subscribers = mesh_markers_->get_subscription_count();
    if (!subscribers) {
      native_mesh_sub_.reset();
      mesh_marker_subscribers_ = 0;
      mesh_subscriber_gids_.clear();
      mesh_snapshot_pending_ = false;
      return;
    }
    if (subscribers > mesh_marker_subscribers_) mesh_snapshot_pending_ = true;
    mesh_marker_subscribers_ = subscribers;
    if (native_mesh_sub_) return;
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_processing_;
    native_mesh_sub_ = create_subscription<nvblox_msgs::msg::Mesh>(
        "~/mesh", rclcpp::QoS(10).reliable(),
        [this, generation = epoch_](const nvblox_msgs::msg::Mesh::ConstSharedPtr mesh) {
          if (generation != epoch_ || !mesh_markers_->get_subscription_count()) return;
          try {
            // A native full-resend indication also schedules an explicit
            // allocated-layer snapshot, including when the input is stationary.
            if (mesh->clear) mesh_snapshot_pending_ = true;
            mesh_markers_->publish(meshMarkersFromMessage(*mesh, center_, size_));
            ++counts_["mesh_marker_updates"];
          } catch (const std::exception &error) {
            ++counts_["invalid_mesh_messages"];
            RCLCPP_ERROR(get_logger(), "Rejected native Mesh: %s", error.what());
          }
        }, options);
  }

  void refreshMeshSubscriberIdentities() {
    if (!mesh_markers_->get_subscription_count()) return;
    // Counts alone miss a disconnect/reconnect with unchanged count. Query
    // graph identities only at the layer cadence, not on every 2 ms tick.
    std::set<std::string> current;
    for (const auto &endpoint : get_subscriptions_info_by_topic(mesh_markers_->get_topic_name())) {
      const auto &gid = endpoint.endpoint_gid();
      std::string id(reinterpret_cast<const char *>(gid.data()), gid.size());
      if (!mesh_subscriber_gids_.count(id)) mesh_snapshot_pending_ = true;
      current.insert(std::move(id));
    }
    mesh_subscriber_gids_ = std::move(current);
  }

  void publishMeshSnapshot() {
    auto start = Wall::now();
    static_mapper_->updateMesh(nvblox::UpdateFullLayer::kYes);
    const auto &layer = static_mapper_->mesh_layer();
    std::vector<nvblox::Index3D> selected;
    for (const auto &index : layer.getAllBlockIndices())
      if (meshBlockInRoi(index, layer.block_size(), center_, size_)) selected.push_back(index);
    visualization_msgs::msg::MarkerArray markers;
    if (!selected.empty()) {
      auto serialized = static_mapper_->layer_streamers().serializeAllBlocks(
          layer, selected, *cuda_stream_);
      if (!serialized) throw std::runtime_error("Native Mesh serializer unavailable");
      nvblox::conversions::markerMessageFromSerializedMesh(
          serialized, params_.global_frame.get(), &markers);
    }
    std_msgs::msg::Header header;
    header.frame_id = params_.global_frame.get(); header.stamp = now();
    for (auto &marker : markers.markers) marker.header = header;
    markers.markers.insert(markers.markers.begin(), clearMeshMarker(header));
    mesh_markers_->publish(markers);
    mesh_snapshot_pending_ = false;
    ++counts_["mesh_marker_snapshots"];
    add("mesh_snapshot_readback", seconds(start) * 1000);
  }

  sensor_msgs::msg::PointCloud2 cloud(const std::vector<EsdfPoint> &points) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = esdf_stamp_;
    msg.header.frame_id = params_.global_frame.get();
    msg.height = 1;
    msg.width = points.size();
    msg.point_step = sizeof(EsdfPoint);
    msg.row_step = msg.width * msg.point_step;
    msg.is_dense = true; // Only finite, observed voxels reach these topics.
    const char *names[] = {"x", "y", "z", "rgb", "distance"};
    for (int i = 0; i < 5; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = i * 4;
      f.count = 1;
      f.datatype = i == 3 ? 6 : 7;
      msg.fields.push_back(f);
    }
    msg.data.resize(msg.row_step);
    if (!points.empty())
      std::memcpy(msg.data.data(), points.data(), msg.row_step);
    return msg;
  }
  void resetMap() {
    // Native clear API updates layer-streamer deletions too. All
    // robot-workspace blocks lie outside this zero-radius sphere, unlike direct
    // layer erasure.
    static_mapper_->clearOutsideRadius(nvblox::Vector3f(1e9f, 1e9f, 1e9f), 0.f);
    cuda_stream_->synchronize();
    {
      std::lock_guard<std::mutex> lock(depth_queue_mutex_);
      depth_image_queue_->clear();
    }
    integrate_depth_last_times_.clear();
    {
      std::lock_guard<std::mutex> lock(color_queue_mutex_);
      color_image_queue_->clear();
    }
    integrate_color_last_times_.clear();
    color_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    color_width_ = color_height_ = 0; color_encoding_.clear();
    newest_integrated_depth_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    esdf_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    shape_clearing_last_time_ = now();
    update_esdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    decay_tsdf_last_time_ = clear_map_outside_radius_last_time_ =
        publish_layer_last_time_ = update_esdf_last_time_;
    seen_.clear();
    candidate_count_ = point_count_ = occupied_count_ = distance_count_ = 0;
    allocated_blocks_ = selected_blocks_ = 0;
    exported_version_ = 0;
    budget_ = 1.;
    budget_last_ = 0;
    scheduled_.clear();
    ++epoch_;
    native_mesh_sub_.reset(); // Discard callbacks belonging to the previous epoch.
    mesh_subscriber_gids_.clear();
    camera_path_.clear(); path_dirty_ = false;
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.global_frame.get(); path.header.stamp = now();
    path_->publish(path);
    std_msgs::msg::UInt64 epoch; epoch.data = epoch_; epoch_pub_->publish(epoch);
    visualization_msgs::msg::MarkerArray clear;
    clear.markers.push_back(clearMeshMarker(path.header)); mesh_markers_->publish(clear);
    mesh_snapshot_pending_ = mesh_markers_->get_subscription_count() > 0;
    ++counts_["map_reset_requested"];
  }
  void add(const std::string &name, double ms) {
    auto &q = latencies_[name];
    q.push_back(ms);
    if (q.size() > 512)
      q.pop_front();
  }
  void publishTopicInfo() {
    Json topics = Json::array();
    auto entry = [&](const std::string &name, const std::string &type,
                     const std::string &meaning, const std::string &color) {
      topics.push_back({{"topic", get_fully_qualified_name() + std::string("/") + name},
                        {"type", type}, {"meaning", meaning}, {"color", color}});
    };
    entry("esdf_occupied", "sensor_msgs/msg/PointCloud2", "Observed inside ESDF voxel centers, NOT a complete distance grid", "red #FF3030");
    entry("esdf_distance_band", "sensor_msgs/msg/PointCloud2", "Observed finite ESDF values within export_band_m; distance in meters", "inside red; free yellow -> cyan -> blue as distance increases");
    if (distance_) entry("esdf_distance", "sensor_msgs/msg/PointCloud2", "Diagnostic: all observed distances in export ROI, includes free space", "same ESDF palette, saturated at export_band_m");
    entry("mesh", "nvblox_msgs/msg/Mesh", "Official incremental triangle mesh; accumulate blocks and apply deletes/clear", "fused camera RGB; missing color may be gray");
    entry("mesh_markers", "visualization_msgs/msg/MarkerArray", "Standard RViz view of the same surface mesh, demand driven", "fused camera RGB");
    entry("color_layer_marker", "visualization_msgs/msg/Marker", "Official TSDF surface-filtered colored voxels; requires RGB integration and a subscriber", "fused camera RGB");
    entry("camera_path", "nav_msgs/msg/Path", "Depth-image-time robot FK/hand-eye TF trajectory; no visual SLAM or loop closure", "green #19FF00 in supplied RViz");
    entry("camera_pose", "geometry_msgs/msg/PoseStamped", "Depth optical camera pose in global_frame at accepted depth timestamp", "RViz axes");
    std_msgs::msg::String msg;
    msg.data = Json({{"interface_revision", 3}, {"frame", params_.global_frame.get()},
                     {"color_enabled", params_.use_color.get()},
                     {"expected_color_frame", expected_color_frame_},
                     {"export_band_m", band_}, {"topics", topics}}).dump();
    topic_info_->publish(msg);
  }
  void publishStats() {
    double dt = seconds(last_stats_);
    Json rates, lat;
    for (auto &[k, v] : counts_) {
      rates[k] = (v - last_counts_[k]) / dt;
      last_counts_[k] = v;
    }
    for (auto &[k, q] : latencies_) {
      if (q.empty())
        continue;
      std::vector<double> a(q.begin(), q.end());
      std::sort(a.begin(), a.end());
      double sum = 0;
      for (double x : a)
        sum += x;
      auto pct = [&](double p) {
        double z = (a.size() - 1) * p;
        size_t i = z;
        return a[i] + (a[std::min(i + 1, a.size() - 1)] - a[i]) * (z - i);
      };
      lat[k] = {{"mean_ms", sum / a.size()},
                {"p50_ms", pct(.5)},
                {"p95_ms", pct(.95)},
                {"p99_ms", pct(.99)},
                {"max_ms", a.back()}};
    }
    Json latest = {{"map_version", version_},
                   {"exported_map_version", exported_version_},
                   {"map_epoch", epoch_},
                   {"input_stamp_ns", esdf_stamp_.nanoseconds()},
                   {"voxel_size_m", static_mapper_->voxel_size_m()},
                   {"visited_allocated_voxels", candidate_count_},
                   {"allocated_blocks", allocated_blocks_},
                   {"selected_blocks", selected_blocks_},
                   {"band_points", point_count_},
                   {"occupied_points", occupied_count_},
                   {"distance_points", distance_count_},
                   {"published_unknown_points", 0},
                   {"pose_source", "robot_tf_at_image_stamp"},
                   {"color_enabled", params_.use_color.get()},
                   {"color_integration_size", {color_width_, color_height_}},
                   {"color_encoding", color_encoding_},
                   {"color_stamp_ns", color_stamp_.nanoseconds()},
                   {"color_age_ros_sec", color_stamp_.nanoseconds() > 0 ? Json((now() - color_stamp_).seconds()) : Json(nullptr)},
                   {"color_state", !params_.use_color.get() ? "disabled" :
                       (color_stamp_.nanoseconds() <= 0 ? "waiting_for_valid_rgb_info_tf" :
                        ((now() - color_stamp_).seconds() > max_age_ ? "stale" : "integrating"))},
                   {"full_distance_cloud_enabled", bool(distance_)},
                   {"integration_size", {input_width_, input_height_}},
                   {"output_mode", "allocated_blocks_sparse_v2"}};
    Json report = {{"backend", "official_ros_cpp"},
                   {"interface_revision", 3},
                   {"rates_hz", rates},
                   {"totals", counts_},
                   {"latency", lat},
                   {"map_latest", latest},
                   {"error", nullptr}};
    std_msgs::msg::String msg;
    msg.data = report.dump();
    stats_->publish(msg);
    last_stats_ = Wall::now();
  }
  BlockExporter exporter_;
  nvblox::Vector3f center_, size_;
  nvblox::Transform esdf_pose_;
  double budget_ = 1., budget_last_ = 0.;
  std::map<std::string, double> scheduled_;
  double band_, export_hz_, max_age_;
  std::string expected_frame_, expected_color_frame_, color_encoding_;
  uint64_t version_ = 0, epoch_ = 0, exported_version_ = 0,
           observed_version_ = 0, occupied_version_ = 0, distance_version_ = 0;
  int64_t last_ros_ns_ = 0;
  size_t candidate_count_ = 0, point_count_ = 0, occupied_count_ = 0, distance_count_ = 0;
  size_t allocated_blocks_ = 0, selected_blocks_ = 0, mesh_marker_subscribers_ = 0;
  bool mesh_snapshot_pending_ = false, path_dirty_ = false;
  std::set<std::string> mesh_subscriber_gids_;
  int path_max_;
  double path_hz_, path_min_translation_, path_min_rotation_;
  std::deque<geometry_msgs::msg::PoseStamped> camera_path_;
  Wall::time_point last_path_publish_{};
  int input_width_ = 0, input_height_ = 0;
  int color_width_ = 0, color_height_ = 0;
  rclcpp::Time color_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esdf_stamp_{0, 0, RCL_ROS_TIME};
  Wall::time_point last_stats_ = Wall::now(), last_export_{},
                   last_input_wall_ = Wall::now(),
                   esdf_input_wall_ = Wall::now();
  std::map<int64_t, Wall::time_point> seen_;
  std::map<std::string, uint64_t> counts_, last_counts_;
  std::map<std::string, std::deque<double>> latencies_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr observed_, occupied_, distance_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_, topic_info_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr epoch_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_markers_;
  rclcpp::Subscription<nvblox_msgs::msg::Mesh>::SharedPtr native_mesh_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_, reset_stats_, republish_mesh_;
};
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<NativeMapper>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception &e) {
    std::cerr << "Native adapter failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
