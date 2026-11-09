// SPDX-License-Identifier: Apache-2.0
// Selection follows NVIDIA layer_publishing.cpp: observed && is_inside for
// occupied ESDF visualization. This is an allocated-block traversal, not the
// dense esdfInAabbToMultiArrayMsg planning-service conversion.
#include "block_export.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace {
constexpr size_t kBatchBlocks = 512;
constexpr int kSide = nvblox::EsdfBlock::kVoxelsPerSide;
constexpr int kPerBlock = nvblox::EsdfBlock::kNumVoxels;
void checked(cudaError_t code) {
  if (code != cudaSuccess) throw std::runtime_error(cudaGetErrorString(code));
}
__global__ void gather(const nvblox::EsdfBlock *const *blocks,
                       const nvblox::Index3D *indices, int num_blocks, float v,
                       float min_x, float min_y, float min_z,
                       float max_x, float max_y, float max_z, float band,
                       EsdfPoint *occupied, EsdfPoint *near_surface,
                       EsdfPoint *distance, int *counts) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_blocks * kPerBlock) return;
  int b = k / kPerBlock, local = k % kPerBlock;
  int ix = local / (kSide * kSide), iy = (local / kSide) % kSide, iz = local % kSide;
  const auto &voxel = blocks[b]->voxels[ix][iy][iz];
  // An allocated block can still contain unknown voxels. Do not emit them.
  if (!voxel.observed || !isfinite(voxel.squared_distance_vox) ||
      voxel.squared_distance_vox < 0.f) return;
  float x = (indices[b].x() * kSide + ix + .5f) * v;
  float y = (indices[b].y() * kSide + iy + .5f) * v;
  float z = (indices[b].z() * kSide + iz + .5f) * v;
  if (x < min_x || x >= max_x || y < min_y || y >= max_y || z < min_z || z >= max_z) return;
  float d = sqrtf(voxel.squared_distance_vox) * v * (voxel.is_inside ? -1.f : 1.f);
  if (!isfinite(d)) return;
  float t = fminf(1.f, fabsf(d) / band);
  // Free space: yellow at zero -> cyan at half-band -> blue at the band.
  // Inside stays red, including signed -0 at an inside surface site.
  uint32_t rgb = t < .5f ?
      (uint32_t(255 * (1 - 2 * t)) << 16) | 0x00ff00 | uint32_t(510 * t) :
      (uint32_t(510 * (1 - t)) << 8) | 0x0000ff;
  if (voxel.is_inside) rgb = 0xff3030;
  EsdfPoint p{x, y, z, rgb, d};
  if (occupied && voxel.is_inside) occupied[atomicAdd(counts, 1)] = p;
  if (near_surface && fabsf(d) <= band) near_surface[atomicAdd(counts + 1, 1)] = p;
  if (distance) distance[atomicAdd(counts + 2, 1)] = p;
}
} // namespace

ExportResult BlockExporter::run(const nvblox::EsdfLayer &layer,
    const nvblox::Vector3f &center, const nvblox::Vector3f &size, float band,
    ExportSelection selection, const nvblox::CudaStream &stream) {
  ExportResult result;
  if (!selection.occupied && !selection.band && !selection.distance) return result;
  if (!std::isfinite(band) || band <= 0 || !center.allFinite() ||
      !size.allFinite() || (size.array() <= 0).any())
    throw std::runtime_error("Invalid sparse ESDF export bounds");
  const auto lo = (center - size / 2.f).eval(), hi = (center + size / 2.f).eval();
  const float bs = layer.block_size(), v = layer.voxel_size();
  const auto allocated = layer.getAllBlockIndices();
  result.allocated_blocks = allocated.size();
  std::vector<nvblox::Index3D> selected;
  selected.reserve(allocated.size());
  for (const auto &index : allocated) {
    auto block_lo = (index.cast<float>() * bs).eval();
    auto block_hi = (block_lo + nvblox::Vector3f::Constant(bs)).eval();
    if ((block_hi.array() <= lo.array()).any() || (block_lo.array() >= hi.array()).any()) continue;
    selected.push_back(index);
  }
  result.selected_blocks = selected.size();
  if (selected.empty()) return result;
  counts_.resizeAsync(3, stream);
  // Fixed-size scratch chunks scale with allocated map blocks rather than ROI volume.
  for (size_t offset = 0; offset < selected.size(); offset += kBatchBlocks) {
    size_t count = std::min(kBatchBlocks, selected.size() - offset);
    std::vector<nvblox::Index3D> ids;
    std::vector<const nvblox::EsdfBlock *> pointers;
    ids.reserve(count); pointers.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      const auto &index = selected[offset + i];
      auto block = layer.getBlockAtIndex(index);
      if (block) { ids.push_back(index); pointers.push_back(block.get()); }
    }
    if (ids.empty()) continue;
    const size_t n = ids.size() * kPerBlock;
    result.visited_voxels += n;
    indices_.copyFromAsync(ids, stream);
    blocks_.copyFromAsync(pointers, stream);
    if (selection.occupied) occupied_.resizeAsync(n, stream);
    if (selection.band) band_.resizeAsync(n, stream);
    if (selection.distance) distance_.resizeAsync(n, stream);
    checked(cudaMemsetAsync(counts_.data(), 0, 3 * sizeof(int), stream));
    gather<<<(n + 255) / 256, 256, 0, stream>>>(blocks_.data(), indices_.data(), ids.size(), v,
        lo.x(), lo.y(), lo.z(), hi.x(), hi.y(), hi.z(), band,
        selection.occupied ? occupied_.data() : nullptr,
        selection.band ? band_.data() : nullptr,
        selection.distance ? distance_.data() : nullptr, counts_.data());
    checked(cudaGetLastError());
    int counts[3] = {};
    checked(cudaMemcpyAsync(counts, counts_.data(), sizeof(counts), cudaMemcpyDeviceToHost, stream));
    stream.synchronize();
    auto read = [&](const nvblox::unified_vector<EsdfPoint> &gpu,
                    int length, std::vector<EsdfPoint> *cpu) {
      if (length < 0 || size_t(length) > n) throw std::runtime_error("Invalid sparse compaction count");
      if (!length) return;
      size_t old = cpu->size(); cpu->resize(old + length);
      checked(cudaMemcpyAsync(cpu->data() + old, gpu.data(), length * sizeof(EsdfPoint),
                              cudaMemcpyDeviceToHost, stream));
    };
    if (selection.occupied) read(occupied_, counts[0], &result.occupied);
    if (selection.band) read(band_, counts[1], &result.band);
    if (selection.distance) read(distance_, counts[2], &result.distance);
    // Host vectors and block pointer arrays may be reused only after readback.
    stream.synchronize();
  }
  return result;
}
