#pragma once

#include <cstdint>

#include "pinky_core/common/types.h"

namespace pinky {

class BatteryMonitor {
 public:
  BatteryMonitor(float v_min = 6.0f, float v_max = 8.4f,
                 float low_threshold = 6.8f);

  // Convert raw 12-bit ADC value to BatteryState.
  BatteryState Update(uint16_t adc_value) const;

  bool IsLow(float voltage) const;

  // Raw ADC -> voltage conversion.
  // Formula from pinky_sensor_adc: (adc/4096) * 4.096 / (13.0/28.0)
  static float AdcToVoltage(uint16_t adc_value);

 private:
  float v_min_;
  float v_max_;
  float low_threshold_;
};

}  // namespace pinky
