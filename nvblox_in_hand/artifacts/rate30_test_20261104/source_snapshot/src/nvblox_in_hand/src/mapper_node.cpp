// SPDX-License-Identifier: Apache-2.0
// Project adapter: official NvbloxNode does all fusion, scheduling, streaming and services.
#include <glog/logging.h>
#include <nvblox/core/internal/warmup_cuda.h>
#include <nvblox/serialization/layer_serializer_gpu.h>
#include <nvblox_ros/nvblox_node.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <sstream>
#include "nvblox_in_hand/esdf_view_policy.hpp"

class InHandMapper : public nvblox::NvbloxNode {
 public:
  InHandMapper() : NvbloxNode(rclcpp::NodeOptions(), "nvblox_node") {
    view_enabled_ = declare_parameter("enable_esdf_view", true);
    view_hz_ = declare_parameter("esdf_view_hz", 30.0);
    band_ = declare_parameter("esdf_view_band_m", 0.12);
    if (!std::isfinite(view_hz_) || view_hz_ < 0 || !std::isfinite(band_) || band_ <= 0)
      throw std::invalid_argument("Invalid ESDF view rate or band");
    view_ = create_publisher<visualization_msgs::msg::Marker>("~/esdf_native_band", 1);
    metrics_ = create_publisher<std_msgs::msg::String>("~/metrics", 1);
  }
 protected:
  bool processDepthImage(const ImageTypeVariant & msg) override {
    auto before=newest_integrated_depth_time_;
    bool ok=NvbloxNode::processDepthImage(msg);
    if(newest_integrated_depth_time_>before) ++depth_; return ok;
  }
  bool processColorImage(const ImageTypeVariant & msg) override {
    auto before=latestStamp(integrate_color_last_times_);
    bool ok=NvbloxNode::processColorImage(msg);
    if(latestStamp(integrate_color_last_times_)>before) ++color_; return ok;
  }
  void tick() override {
    NvbloxNode::tick();  // Do not duplicate or reorder the upstream pipeline.
    auto t = Clock::now(); double dt = std::chrono::duration<double>(t - last_metrics_).count();
    if (dt < 1.0) return;
    std::ostringstream s;
    s << "{\"backend\":\"official_ros_cpp\",\"totals\":{\"depth_integrated\":" << depth_ << ",\"color_integrated\":" << color_ << ",\"esdf_updated\":" << esdf_ << "},\"depth_integrated_total\":" << depth_ << ",\"color_integrated_total\":" << color_
      << ",\"esdf_cycles_total\":" << esdf_ << ",\"depth_integrated_wall_hz\":" << (depth_-prev_depth_)/dt
      << ",\"color_integrated_wall_hz\":" << (color_-prev_color_)/dt
      << ",\"esdf_cycles_wall_hz\":" << (esdf_-prev_esdf_)/dt
      << ",\"esdf_process_host_ms\":" << esdf_host_ms_
      << ",\"esdf_view_host_ms\":" << view_ms_ << ",\"esdf_view_points\":" << view_points_
      << ",\"esdf_export_stats_valid\":" << (export_stats_valid_ ? "true" : "false")
      << ",\"esdf_export_allocated_blocks\":" << export_blocks_
      << ",\"esdf_export_allocated_voxels\":" << export_voxels_
      << ",\"esdf_export_observed_voxels\":" << export_observed_
      << ",\"esdf_export_observed_inside_voxels\":" << export_inside_
      << ",\"voxel_size_m\":" << static_mapper_->voxel_size_m()
      << ",\"sampling\":\"native_allocated_observed_voxels\",\"gpu_completion_measured\":false}";
    std_msgs::msg::String out; out.data=s.str(); metrics_->publish(out);
    prev_depth_=depth_; prev_color_=color_; prev_esdf_=esdf_; last_metrics_=t;
  }
  void processEsdf() override {
    auto began = Clock::now();
    NvbloxNode::processEsdf(); ++esdf_;
    esdf_host_ms_ = std::chrono::duration<double,std::milli>(Clock::now()-began).count();
    if (!view_enabled_ || view_hz_<=0 || view_->get_subscription_count()==0 ||
        std::chrono::duration<double>(Clock::now()-last_view_).count() < 1./view_hz_) return;
    began=Clock::now();
    // Serialize actual allocated ESDF blocks using the upstream serializer.
    // This optional visualization incurs a synchronized host readback, not ESDF construction.
    auto & layer = static_mapper_->esdf_layer();
    auto data = serializer_.serialize(layer, layer.getAllBlockIndices(), *cuda_stream_);
    visualization_msgs::msg::Marker out;
    out.header.frame_id=params_.global_frame.get(); out.header.stamp=now();
    out.ns="observed_native_esdf_band"; out.id=0;
    out.type=visualization_msgs::msg::Marker::CUBE_LIST;
    out.action=visualization_msgs::msg::Marker::ADD; out.pose.orientation.w=1.;
    out.scale.x=out.scale.y=out.scale.z=layer.voxel_size()*.95;
    out.color.a=1.;
    constexpr int side=nvblox::EsdfBlock::kVoxelsPerSide;
    uint64_t observed_count=0,inside_count=0;
    for (size_t b=0;b<data->block_indices.size();++b) {
      int offset=data->block_offsets[b];
      for (int x=0;x<side;++x) for(int y=0;y<side;++y) for(int z=0;z<side;++z) {
        const auto & v=data->voxels[offset+z+side*y+side*side*x];
        if(v.observed) { ++observed_count; if(v.is_inside) ++inside_count; }
        auto signed_distance=in_hand::observedDistance(v.observed,v.is_inside,v.squared_distance_vox,layer.voxel_size(),band_);
        if (!signed_distance) continue;
        float distance=std::abs(*signed_distance);
        auto pos=nvblox::getCenterPositionFromBlockIndexAndVoxelIndex(layer.block_size(),data->block_indices[b],{x,y,z});
        geometry_msgs::msg::Point pt; pt.x=pos.x();pt.y=pos.y();pt.z=pos.z();out.points.push_back(pt);
        std_msgs::msg::ColorRGBA c; c.a=1.;
        if(v.is_inside){c.r=1.;c.g=.1f;} else {c.g=1.f-distance/band_;c.b=distance/band_;}
        out.colors.push_back(c);
      }
    }
    export_blocks_=data->block_indices.size();
    export_voxels_=export_blocks_*side*side*side;
    export_observed_=observed_count;export_inside_=inside_count;export_stats_valid_=true;
    view_points_=out.points.size();view_->publish(out);last_view_=Clock::now();
    view_ms_=std::chrono::duration<double,std::milli>(last_view_-began).count();
  }
 private:
  template<typename Map> static int64_t latestStamp(const Map & map) {
    int64_t stamp=0; for(const auto & item:map) if(item.second.nanoseconds()>stamp) stamp=item.second.nanoseconds(); return stamp;
  }
  using Clock=std::chrono::steady_clock;
  bool view_enabled_;double view_hz_,band_,esdf_host_ms_=0,view_ms_=0;
  uint64_t depth_=0,color_=0,esdf_=0,prev_depth_=0,prev_color_=0,prev_esdf_=0,view_points_=0;
  bool export_stats_valid_=false;
  uint64_t export_blocks_=0,export_voxels_=0,export_observed_=0,export_inside_=0;
  Clock::time_point last_metrics_=Clock::now(),last_view_{};
  nvblox::EsdfLayerSerializerGpu serializer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr view_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_;
};
int main(int argc,char **argv) {
  google::InitGoogleLogging(argv[0]);FLAGS_alsologtostderr=true;
  rclcpp::init(argc,argv);nvblox::warmupCuda();
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node=std::make_shared<InHandMapper>();exec.add_node(node);exec.spin();
  rclcpp::shutdown();return 0;
}
