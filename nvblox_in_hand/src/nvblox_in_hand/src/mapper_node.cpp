// SPDX-License-Identifier: Apache-2.0
// Opt-in diagnostic backend. Production bringup loads the official component
// directly; this adapter adds native voxel inspection and counters only.
#include <glog/logging.h>
#include <nvblox/core/internal/warmup_cuda.h>
#include <nvblox_ros/nvblox_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <sstream>
#include <cstring>
#include <limits>
#include "nvblox_in_hand/esdf_pointcloud_exporter.hpp"

class InHandMapper : public nvblox::NvbloxNode {
 public:
  InHandMapper() : NvbloxNode(rclcpp::NodeOptions(), "nvblox_node") {
    view_enabled_ = declare_parameter("enable_esdf_view", true);
    view_hz_ = declare_parameter("esdf_view_hz", 30.0);
    band_ = declare_parameter("esdf_view_band_m", 0.12);
    const auto mode = declare_parameter("esdf_view_mode", std::string("observed"));
    if (mode == "inside" || mode == "surface") selection_=in_hand::EsdfPointSelection::kInside;
    else if (mode == "band") selection_=in_hand::EsdfPointSelection::kBand;
    else if (mode == "observed") selection_=in_hand::EsdfPointSelection::kObserved;
    else throw std::invalid_argument("esdf_view_mode must be observed, band or inside");
    if (mode == "surface") RCLCPP_WARN(get_logger(),
      "surface is a deprecated alias for inside; it is not a reconstructed surface");
    if (!std::isfinite(view_hz_) || view_hz_ < 0 || !std::isfinite(band_) || band_ <= 0)
      throw std::invalid_argument("Invalid ESDF view rate or band");
    view_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/esdf_points", 1);
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
    publishEsdfPoints();
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
      << ",\"sampling\":\"native_gpu_compacted_esdf_voxels\",\"gpu_completion_measured\":false}";
    std_msgs::msg::String out; out.data=s.str(); metrics_->publish(out);
    prev_depth_=depth_; prev_color_=color_; prev_esdf_=esdf_; last_metrics_=t;
  }
  void processEsdf() override {
    const auto began = Clock::now();
    NvbloxNode::processEsdf(); ++esdf_;
    esdf_input_stamp_=newest_integrated_depth_time_;
    esdf_host_ms_ = std::chrono::duration<double,std::milli>(Clock::now()-began).count();
  }
 private:
  void publishEsdfPoints() {
    const auto began = Clock::now();
    if (!view_enabled_ || view_hz_<=0 || view_->get_subscription_count()==0 ||
        params_.esdf_mode != nvblox::EsdfMode::k3D || esdf_input_stamp_.nanoseconds()<=0 ||
        exported_version_==esdf_ ||
        std::chrono::duration<double>(began-last_view_).count() < 1./view_hz_) return;
    // Start-to-start cadence, rather than adding export duration to each period.
    last_view_=began;
    auto data=exporter_.run(static_mapper_->esdf_layer(),selection_,band_,*cuda_stream_);
    if(data.points.size()>std::numeric_limits<uint32_t>::max()/sizeof(in_hand::EsdfDisplayPoint))
      throw std::runtime_error("ESDF cloud exceeds PointCloud2 row size");
    sensor_msgs::msg::PointCloud2 out;
    out.header.frame_id=params_.global_frame.get();out.header.stamp=esdf_input_stamp_;
    out.height=1;out.width=data.points.size();out.is_bigendian=false;out.is_dense=true;
    out.point_step=sizeof(in_hand::EsdfDisplayPoint);out.row_step=out.width*out.point_step;
    auto field=[&](const std::string &name,uint32_t offset,uint8_t type) {
      sensor_msgs::msg::PointField f;f.name=name;f.offset=offset;f.datatype=type;f.count=1;
      out.fields.push_back(f);
    };
    using F=sensor_msgs::msg::PointField;
    field("x",0,F::FLOAT32);field("y",4,F::FLOAT32);field("z",8,F::FLOAT32);
    field("distance",12,F::FLOAT32);field("intensity",12,F::FLOAT32);field("rgb",16,F::UINT32);
    field("observed",20,F::UINT8);field("is_inside",21,F::UINT8);field("is_site",22,F::UINT8);
    out.data.resize(out.row_step);
    if(!out.data.empty()) std::memcpy(out.data.data(),data.points.data(),out.data.size());
    export_blocks_=data.blocks;export_voxels_=data.allocated_voxels;
    export_observed_=data.observed_voxels;export_inside_=data.inside_voxels;export_stats_valid_=true;
    view_points_=out.width;view_->publish(out);exported_version_=esdf_;
    view_ms_=std::chrono::duration<double,std::milli>(Clock::now()-began).count();
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
  in_hand::EsdfPointSelection selection_=in_hand::EsdfPointSelection::kObserved;
  in_hand::EsdfPointCloudExporter exporter_;
  uint64_t exported_version_=0;
  rclcpp::Time esdf_input_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr view_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_;
};
int main(int argc,char **argv) {
  google::InitGoogleLogging(argv[0]);FLAGS_alsologtostderr=true;
  rclcpp::init(argc,argv);nvblox::warmupCuda();
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node=std::make_shared<InHandMapper>();exec.add_node(node);exec.spin();
  rclcpp::shutdown();return 0;
}
