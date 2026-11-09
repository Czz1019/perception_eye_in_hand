// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>
#include <cstdint>
#include <nvblox/core/unified_vector.h>
#include <nvblox/map/common_names.h>
#include <vector>

namespace in_hand {
// Diagnostic selection only. Inside is not a reconstructed surface and is not
// the complete ESDF. kSurface is retained as a source-compatibility alias.
enum class EsdfPointSelection { kInside = 0, kSurface = kInside, kBand = 1, kObserved = 2 };
struct EsdfDisplayPoint {
  float x, y, z, distance;
  uint32_t rgb;
  uint8_t observed, is_inside, is_site, reserved;
};
static_assert(sizeof(EsdfDisplayPoint) == 24);
static_assert(offsetof(EsdfDisplayPoint, distance) == 12);
static_assert(offsetof(EsdfDisplayPoint, rgb) == 16);
static_assert(offsetof(EsdfDisplayPoint, observed) == 20);
struct EsdfCloudExport {
  std::vector<EsdfDisplayPoint> points;
  size_t blocks = 0, allocated_voxels = 0, observed_voxels = 0, inside_voxels = 0;
};
class EsdfPointCloudExporter {
 public:
  EsdfCloudExport run(const nvblox::EsdfLayer & layer, EsdfPointSelection selection,
                      float band_m, const nvblox::CudaStream & stream);
 private:
  nvblox::unified_vector<const nvblox::EsdfBlock *> blocks_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<nvblox::Index3D> indices_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<EsdfDisplayPoint> points_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<int> counts_{nvblox::MemoryType::kDevice};
};
}  // namespace in_hand
