#include "pinky_core/core/battery_monitor.h"

#include <algorithm>

#include "pinky_core/common/constants.h"

namespace pinky {

BatteryMonitor::BatteryMonitor(float v_min, float v_max, float low_threshold)
    : v_min_(v_min), v_max_(v_max), low_threshold_(low_threshold) {}

float BatteryMonitor::AdcToVoltage(uint16_t adc_value) {
  // Voltage divider: R1=15k, R2=13k → ratio = 13/(13+15) = 13/28
  // ADC ref = 4.096V, 12-bit (4096 levels)
  return (static_cast<float>(adc_value) / 4096.0f) * 4.096f / (13.0f / 28.0f);
}

BatteryState BatteryMonitor::Update(uint16_t adc_value) const {
  BatteryState state;
  state.stamp = Timestamp::Now();
  state.voltage = AdcToVoltage(adc_value);

  // Linear mapping: v_min -> 0%, v_max -> 100%
  float range = v_max_ - v_min_;
  if (range > 0.0f) {
    state.percentage =
        Clamp((state.voltage - v_min_) / range * 100.0f, 0.0f, 100.0f);
  }

  state.status = IsLow(state.voltage) ? 0 : 1;
  return state;
}

bool BatteryMonitor::IsLow(float voltage) const {
  return voltage <= low_threshold_;
}

}  // namespace pinky
