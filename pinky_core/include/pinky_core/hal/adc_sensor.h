#pragma once

#include <string>

#include "pinky_core/hal/interfaces.h"

namespace pinky {

class AdcSensor : public IAdcDriver {
 public:
  struct Config {
    std::string interface{"/dev/i2c-1"};
    int address{0x08};
  };

  explicit AdcSensor(const Config& config);
  ~AdcSensor() override;

  bool Init() override;
  bool ReadAll(uint16_t& ch0, uint16_t& ch1, uint16_t& ch2, uint16_t& ch3, uint16_t& ch4) override;

 private:
  Config config_;
  int fd_{-1};
};

}  // namespace pinky
