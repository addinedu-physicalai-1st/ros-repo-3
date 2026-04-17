#pragma once

#include <string>

#include "pinky_core/hal/interfaces.h"

namespace pinky {

class Bno055Imu : public IImuDriver {
 public:
  struct Config {
    std::string interface{"/dev/i2c-0"};
    int address{0x28};
  };

  explicit Bno055Imu(const Config& config);
  ~Bno055Imu() override;

  bool Init() override;
  bool ReadData(ImuData& data) override;

 private:
  Config config_;
  int fd_{-1};
};

}  // namespace pinky
