#include "pinky_core/core/lidar_processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pinky {

LidarProcessor::LidarProcessor(int num_sectors, float max_range)
    : num_sectors_(num_sectors), max_range_(max_range) {}

LidarSectors LidarProcessor::Process(const LidarScan& scan) const {
  return Process(scan.ranges.data(), static_cast<int>(scan.ranges.size()));
}

LidarSectors LidarProcessor::Process(const float* ranges,
                                     int num_points) const {
  LidarSectors result{};
  if (num_points <= 0) {
    return result;
  }

  // 1. Clean: copy and replace NaN/Inf
  std::vector<float> cleaned(num_points);
  for (int i = 0; i < num_points; ++i) {
    float v = ranges[i];
    if (std::isnan(v)) {
      cleaned[i] = 0.0f;
    } else if (std::isinf(v)) {
      cleaned[i] = (v > 0.0f) ? max_range_ : 0.0f;
    } else {
      cleaned[i] = v;
    }
  }

  // 2. Roll by n/2 (front-center alignment)
  //    Python: np.roll(ranges, n // 2)
  //    C++: std::rotate left by (n - n/2) = rotate right by n/2
  int roll = num_points / 2;
  std::rotate(cleaned.begin(), cleaned.end() - roll, cleaned.end());

  // 3. Sector min-pooling (24 sectors)
  int sector_size = std::max(1, num_points / num_sectors_);
  for (int i = 0; i < num_sectors_; ++i) {
    int start = i * sector_size;
    int end = (i < num_sectors_ - 1) ? start + sector_size : num_points;
    end = std::min(end, num_points);

    float sector_min = max_range_;
    for (int j = start; j < end; ++j) {
      if (cleaned[j] > 0.10f) { // ignore noise below RPLIDAR min range (~15cm)
        sector_min = std::min(sector_min, cleaned[j]);
      }
    }

    // 4. Normalize by max_range
    result.sectors[i] = sector_min / max_range_;
  }

  result.stamp = Timestamp::Now();
  return result;
}

}  // namespace pinky
