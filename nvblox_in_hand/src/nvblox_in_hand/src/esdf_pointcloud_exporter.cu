// SPDX-License-Identifier: Apache-2.0
// Native allocated-block export; no synthetic query lattice and no unknown padding.
#include "nvblox_in_hand/esdf_pointcloud_exporter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace in_hand {
namespace {
constexpr size_t kBatchBlocks = 512;
constexpr int kSide = nvblox::EsdfBlock::kVoxelsPerSide;
constexpr int kPerBlock = nvblox::EsdfBlock::kNumVoxels;
void check(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
__global__ void compactEsdf(const nvblox::EsdfBlock * const * blocks,
    const nvblox::Index3D * indices, int block_count, float voxel_size,
    int selection, float band, EsdfDisplayPoint * output, int * counts) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= block_count * kPerBlock) return;
  const int block = i / kPerBlock, local = i % kPerBlock;
  const int x = local / (kSide*kSide), y = (local/kSide)%kSide, z = local%kSide;
  const auto & v = blocks[block]->voxels[x][y][z];
  if (!v.observed) return;
  atomicAdd(counts + 1, 1);
  if (v.is_inside) atomicAdd(counts + 2, 1);
  if (!isfinite(v.squared_distance_vox) || v.squared_distance_vox < 0.f) return;
  const float abs_d = sqrtf(v.squared_distance_vox)*voxel_size;
  if (!isfinite(abs_d)) return;
  if (selection == int(EsdfPointSelection::kInside) && !v.is_inside) return;
  if (selection == int(EsdfPointSelection::kBand) && abs_d > band) return;
  const float d = v.is_inside ? -abs_d : abs_d;
  const float t = fminf(1.f, abs_d/band);
  const uint32_t rgb = v.is_inside ? 0xff3030u :
      (uint32_t(255.f*(1.f-t)) << 8) | uint32_t(255.f*t);
  const EsdfDisplayPoint point{
    (indices[block].x()*kSide+x+.5f)*voxel_size,
    (indices[block].y()*kSide+y+.5f)*voxel_size,
    (indices[block].z()*kSide+z+.5f)*voxel_size, d, rgb,
    uint8_t(v.observed), uint8_t(v.is_inside), uint8_t(v.is_site), 0};
  output[atomicAdd(counts, 1)] = point;
}
}  // namespace

EsdfCloudExport EsdfPointCloudExporter::run(const nvblox::EsdfLayer & layer,
    EsdfPointSelection selection, float band, const nvblox::CudaStream & stream) {
  if (!std::isfinite(band) || band <= 0.f || int(selection) < 0 || int(selection) > 2)
    throw std::invalid_argument("Invalid ESDF point selection");
  EsdfCloudExport result;
  const auto allocated = layer.getAllBlockIndices();
  result.blocks = allocated.size();
  result.allocated_voxels = allocated.size()*kPerBlock;
  counts_.resizeAsync(3, stream);
  for (size_t offset=0; offset<allocated.size(); offset+=kBatchBlocks) {
    std::vector<const nvblox::EsdfBlock *> pointers;
    std::vector<nvblox::Index3D> ids;
    const size_t end = std::min(allocated.size(),offset+kBatchBlocks);
    pointers.reserve(end-offset); ids.reserve(end-offset);
    for (size_t i=offset; i<end; ++i) {
      const auto block = layer.getBlockAtIndex(allocated[i]);
      if (block) { pointers.push_back(block.get()); ids.push_back(allocated[i]); }
    }
    if (ids.empty()) continue;
    const size_t capacity = ids.size()*kPerBlock;
    blocks_.copyFromAsync(pointers,stream); indices_.copyFromAsync(ids,stream);
    points_.resizeAsync(capacity,stream);
    check(cudaMemsetAsync(counts_.data(),0,3*sizeof(int),stream));
    compactEsdf<<<(capacity+255)/256,256,0,stream>>>(blocks_.data(),indices_.data(),
        ids.size(),layer.voxel_size(),int(selection),band,points_.data(),counts_.data());
    check(cudaGetLastError());
    int counts[3] = {};
    check(cudaMemcpyAsync(counts,counts_.data(),sizeof(counts),cudaMemcpyDeviceToHost,stream));
    stream.synchronize();
    if (counts[0] < 0 || size_t(counts[0]) > capacity)
      throw std::runtime_error("Invalid compacted ESDF count");
    result.observed_voxels += counts[1]; result.inside_voxels += counts[2];
    if (counts[0]) {
      const size_t old_size = result.points.size();
      result.points.resize(old_size+counts[0]);
      check(cudaMemcpyAsync(result.points.data()+old_size,points_.data(),
          counts[0]*sizeof(EsdfDisplayPoint),cudaMemcpyDeviceToHost,stream));
      stream.synchronize();
    }
  }
  return result;
}
}  // namespace in_hand
