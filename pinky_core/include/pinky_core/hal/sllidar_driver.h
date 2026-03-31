#pragma once

#include <memory>
#include <string>

#include "pinky_core/hal/interfaces.h"

// Forward declaration for Slamtec SDK types to avoid exposing them in headers
namespace sl {
class ILidarDriver;
}

namespace pinky {

class SllidarDriver : public ILidarDriver {
 public:
  struct Config {
    std::string port{"/dev/ttyAMA0"};
    int baudrate{1000000};
  };

  explicit SllidarDriver(const Config& config);
  ~SllidarDriver() override;

  bool Init() override;
  bool StartScan() override;
  void StopScan() override;
  bool GetScan(LidarScan& scan) override;

 private:
  Config config_;
  sl::ILidarDriver* drv_{nullptr};
};

}  // namespace pinky
