// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <optional>
namespace in_hand {
// Display-only selection of real ESDF voxels. Unknown is never treated as occupied/free.
inline std::optional<float> observedDistance(bool observed, bool inside,
    float squared_distance_vox, float voxel_size_m, float band_m) {
  if (!observed || !std::isfinite(squared_distance_vox) || squared_distance_vox<0 ||
      !std::isfinite(voxel_size_m) || voxel_size_m<=0 || !std::isfinite(band_m) || band_m<=0)
    return std::nullopt;
  float distance=std::sqrt(squared_distance_vox)*voxel_size_m;
  if (distance>band_m) return std::nullopt;
  return inside ? -distance : distance;
}
}
