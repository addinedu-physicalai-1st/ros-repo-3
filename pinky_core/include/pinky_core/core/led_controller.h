#pragma once

#include <cstdint>
#include <tuple>

namespace pinky {

// LED animation state machine.
// Modes: 0=off, 1=solid on, 2=blink, 3=dimming.
// Call Tick() at 100Hz (10ms interval) to advance animation.
class LedController {
 public:
  void SetMode(uint8_t mode, uint8_t r, uint8_t g, uint8_t b,
               uint16_t time_ms);

  // Returns ARGB color (0xAARRGGBB) for this frame.
  uint32_t Tick();

  uint8_t mode() const { return mode_; }

  // Color conversions
  static std::tuple<double, double, double> RgbToHsv(uint8_t r, uint8_t g,
                                                     uint8_t b);
  static std::tuple<uint8_t, uint8_t, uint8_t> HsvToRgb(double h, double s,
                                                         double v);

 private:
  uint8_t mode_{0};
  uint8_t r_{0};
  uint8_t g_{0};
  uint8_t b_{0};
  uint16_t time_ms_{0};

  int16_t time_index_{0};
  uint8_t direction_{0};

  // HSV values for dimming mode
  double h_{0.0};
  double s_{0.0};
  double v_{0.0};
};

}  // namespace pinky
