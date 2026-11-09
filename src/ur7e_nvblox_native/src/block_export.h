// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>
#include <nvblox/core/unified_vector.h>
#include <nvblox/map/common_names.h>
#include <vector>

struct EsdfPoint {
  float x, y, z;
  uint32_t rgb;
  float distance;
};
static_assert(sizeof(EsdfPoint) == 20);

struct ExportSelection {
  bool occupied = false; // observed && is_inside, matching official ESDF filter definition
  bool band = false;     // observed && abs(distance) <= band
  bool distance = false; // every observed finite distance, including free space
};
struct ExportResult {
  std::vector<EsdfPoint> occupied, band, distance;
  size_t allocated_blocks = 0, selected_blocks = 0, visited_voxels = 0;
};

// Enumerates ALLOCATED blocks, never a synthetic ROI grid. The ROI only crops
// actual map data. A batch caps scratch allocation; there is no unknown padding.
class BlockExporter {
public:
  ExportResult run(const nvblox::EsdfLayer &, const nvblox::Vector3f &center,
                   const nvblox::Vector3f &size, float band,
                   ExportSelection selection, const nvblox::CudaStream &);
private:
  nvblox::unified_vector<const nvblox::EsdfBlock *> blocks_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<nvblox::Index3D> indices_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<EsdfPoint> occupied_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<EsdfPoint> band_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<EsdfPoint> distance_{nvblox::MemoryType::kDevice};
  nvblox::unified_vector<int> counts_{nvblox::MemoryType::kDevice};
};
