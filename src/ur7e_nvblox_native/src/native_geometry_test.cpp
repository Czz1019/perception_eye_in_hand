// SPDX-License-Identifier: Apache-2.0
// Standalone sparse-v2 regression. Building it does not execute CUDA tests.
#include "block_export.h"
#include "mesh_visualization.h"
#include <cmath>
#include <glog/logging.h>
#include <iostream>
#include <nvblox/mapper/mapper.h>
#include <nvblox/serialization/layer_serializer_gpu.h>
#include <nvblox_ros/conversions/esdf_and_gradients_conversions.hpp>
#include <set>
#include <stdexcept>
void require(bool good, const char *message) {
  if (!good) throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  try {
    auto stream = std::make_shared<nvblox::CudaStreamOwning>();
    nvblox::Mapper mapper(.05, nvblox::MemoryType::kDevice,
                          nvblox::ProjectiveLayerType::kTsdf, stream);
    std::vector<float> depth(80 * 60, 1.f);
    nvblox::DepthImage image(nvblox::MemoryType::kDevice);
    image.copyFromAsync(60, 80, depth.data(), *stream);
    nvblox::Camera camera(70, 70, 40, 30, 80, 60);
    nvblox::Transform pose = nvblox::Transform::Identity();
    pose.translation() = nvblox::Vector3f(.2, .1, .3);
    for (int i = 0; i < 4; ++i) mapper.integrateDepth(image, pose, camera);
    mapper.updateEsdf(); stream->synchronize();
    BlockExporter exporter;
    const nvblox::Vector3f center(.2, .1, .9), size(2., 2., 1.2);
    auto result = exporter.run(mapper.esdf_layer(), center, size, .15,
                               {true, true, true}, *stream);
    require(result.band.size() > 100 && !result.occupied.empty(), "Missing plane observations");
    auto request = std::make_shared<nvblox_msgs::srv::EsdfAndGradients::Request>();
    request->use_aabb = true;
    const auto lo = (center - size / 2.f).eval();
    request->aabb_min_m.x = lo.x(); request->aabb_min_m.y = lo.y(); request->aabb_min_m.z = lo.z();
    request->aabb_size_m.x = size.x(); request->aabb_size_m.y = size.y(); request->aabb_size_m.z = size.z();
    auto reference = std::make_shared<nvblox_msgs::srv::EsdfAndGradients::Response>();
    nvblox::conversions::EsdfAndGradientsConverter official;
    official.getEsdfAndGradientResponse(mapper.esdf_layer(), 1e6, request, reference, *stream);
    require(reference->success, "Official conversion failed");
    const auto &array = reference->esdf_and_gradients;
    const int dx = array.layout.dim[0].size, dy = array.layout.dim[1].size, dz = array.layout.dim[2].size;
    const float v = reference->voxel_size_m;
    auto linear = [&](const EsdfPoint &p) {
      int x = std::lround((p.x - reference->origin_m.x) / v - .5);
      int y = std::lround((p.y - reference->origin_m.y) / v - .5);
      int z = std::lround((p.z - reference->origin_m.z) / v - .5);
      require(x >= 0 && y >= 0 && z >= 0 && x < dx && y < dy && z < dz, "Point outside reference");
      return (size_t(x) * dy + y) * dz + z;
    };
    std::set<size_t> seen;
    double maximum = 0;
    size_t free = 0, near_surface = 0;
    for (auto p : result.distance) {
      require(std::isfinite(p.distance) && std::abs(p.distance) < 1e5, "Sparse export contains unknown");
      for (float c : {p.x, p.y, p.z})
        require(std::abs(c / v - .5f - std::round(c / v - .5f)) < 1e-4, "Non-native centre");
      auto index = linear(p);
      require(seen.insert(index).second, "Duplicate native voxel");
      maximum = std::max(maximum, double(std::abs(array.data[index] - p.distance)));
      if (p.distance > .075) ++free;
      if (std::abs(p.z - 1.3) < .05 && std::abs(p.distance) < .075) ++near_surface;
    }
    require(maximum < 1e-5 && free > 10 && near_surface > 10, "Sparse/reference geometry mismatch");
    size_t expected_known = 0;
    for (int x = 0; x < dx; ++x) for (int y = 0; y < dy; ++y) for (int z = 0; z < dz; ++z) {
      const float px = reference->origin_m.x + (x + .5f) * v;
      const float py = reference->origin_m.y + (y + .5f) * v;
      const float pz = reference->origin_m.z + (z + .5f) * v;
      const auto hi = (center + size / 2.f).eval();
      if (px < lo.x() || px >= hi.x() || py < lo.y() || py >= hi.y() || pz < lo.z() || pz >= hi.z()) continue;
      float d = array.data[(size_t(x) * dy + y) * dz + z];
      if (std::isfinite(d) && std::abs(d) < 1e5) ++expected_known;
    }
    require(expected_known == result.distance.size(), "Allocated traversal omits known ROI voxels");
    for (auto p : result.band)
      require(std::abs(p.distance) <= .15001 && seen.count(linear(p)), "Invalid distance band");
    for (auto p : result.occupied)
      require(p.distance <= 0 && seen.count(linear(p)), "Invalid occupied voxel");
    // Signed zero cannot distinguish inside/outside on the surface. Compare
    // against the actual native flag as well as comparing signed distances.
    nvblox::EsdfLayerSerializerGpu serializer;
    auto native = serializer.serialize(mapper.esdf_layer(),
        mapper.esdf_layer().getAllBlockIndices(), *stream);
    std::set<size_t> expected_inside, actual_inside;
    const auto hi = (center + size / 2.f).eval();
    size_t native_unknown = 0;
    for (size_t b = 0; b < native->block_indices.size(); ++b) {
      const auto &index = native->block_indices[b];
      const int begin = native->block_offsets[b];
      for (int j = 0; j < nvblox::EsdfBlock::kNumVoxels; ++j) {
        const auto &voxel = native->voxels[begin + j];
        if (!voxel.observed) ++native_unknown;
        if (!voxel.observed || !voxel.is_inside ||
            !std::isfinite(voxel.squared_distance_vox) || voxel.squared_distance_vox < 0) continue;
        const int side = nvblox::EsdfBlock::kVoxelsPerSide;
        EsdfPoint p{(index.x() * side + j / (side * side) + .5f) * v,
                    (index.y() * side + (j / side) % side + .5f) * v,
                    (index.z() * side + j % side + .5f) * v, 0, 0};
        if (p.x >= lo.x() && p.x < hi.x() && p.y >= lo.y() && p.y < hi.y() &&
            p.z >= lo.z() && p.z < hi.z()) expected_inside.insert(linear(p));
      }
    }
    for (const auto &p : result.occupied) actual_inside.insert(linear(p));
    require(actual_inside == expected_inside, "Occupied export differs from native observed/is_inside flags");
    // Exercise unknown cells in REAL allocated blocks. Do not instantiate
    // BlockLayer::allocateBlockAtIndexAsync in a host-only .cpp: the installed
    // CUDA library's Thrust pair ABI includes its CUDA architecture list.
    require(native_unknown > 100, "Reference map does not cover unknown voxel filtering");
    nvblox::Mapper empty(.05);
    auto blank = exporter.run(empty.esdf_layer(), {0, 0, 0}, {1, 1, 1}, .15,
                              {true, true, true}, *stream);
    require(blank.allocated_blocks == 0 && blank.visited_voxels == 0 &&
            blank.occupied.empty() && blank.band.empty() && blank.distance.empty(),
            "Empty map produces placeholder points or stale output");
    // No allocated blocks near this ROI: increasing its volume must not fill it.
    auto away = exporter.run(mapper.esdf_layer(), {100, 100, 100}, {10, 10, 10}, .15,
                             {true, true, true}, *stream);
    require(away.selected_blocks == 0 && away.visited_voxels == 0 && away.distance.empty(),
            "Empty ROI still scans a synthetic grid");
    nvblox_msgs::msg::Mesh mesh; mesh.block_size_m = .4; mesh.clear = true;
    mesh.block_indices.resize(1); mesh.blocks.resize(1);
    mesh.blocks[0].vertices.resize(3);
    mesh.blocks[0].vertices[1].x = .1; mesh.blocks[0].vertices[2].y = .1;
    mesh.blocks[0].triangles = {0, 1, 2};
    auto markers = meshMarkersFromMessage(mesh, {0, 0, 0}, {1, 1, 1});
    require(markers.markers.size() == 2 && markers.markers[1].points.size() == 3,
            "Mesh triangle expansion or clear failed");
    mesh.clear = false; mesh.blocks[0].triangles.clear();
    markers = meshMarkersFromMessage(mesh, {0, 0, 0}, {1, 1, 1});
    require(markers.markers[0].action == visualization_msgs::msg::Marker::DELETE,
            "Empty mesh block does not delete old geometry");
    std::cout << "PASS sparse_v2 official_distance_max_error=" << maximum
              << " observed=" << result.distance.size() << " unknown_padding=0\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl; return 1;
  }
  return 0;
}
