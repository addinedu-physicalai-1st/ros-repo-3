#pragma once

#include "pinky_core/common/constants.h"
#include "pinky_core/common/types.h"

namespace pinky {

// Preprocesses raw LiDAR scan into 24 normalized sectors.
// Pipeline matches Python training code exactly:
//   1. NaN/Inf clean
//   2. Roll by n/2 (front-center alignment)
//   3. 24-sector min-pooling
//   4. Normalize by max_range
class LidarProcessor {
 public:
  explicit LidarProcessor(int num_sectors = kLidarSectors,
                          float max_range = kMaxLidarDist,
                          float min_range_filter = kLidarMinRange);

  // Process raw scan -> 24 normalized sectors in [0, 1].
  LidarSectors Process(const LidarScan& scan) const;

  // Process a raw float array directly.
  LidarSectors Process(const float* ranges, int num_points) const;

  int num_sectors() const { return num_sectors_; }
  float max_range() const { return max_range_; }

 private:
  int num_sectors_;
  float max_range_;
  float min_range_filter_;
};

}  // namespace pinky
