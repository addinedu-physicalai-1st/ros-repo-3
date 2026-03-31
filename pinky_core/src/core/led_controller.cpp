#include "pinky_core/core/led_controller.h"

#include <algorithm>
#include <cmath>

namespace pinky {

void LedController::SetMode(uint8_t mode, uint8_t r, uint8_t g, uint8_t b,
                            uint16_t time_ms) {
  mode_ = mode;
  r_ = r;
  g_ = g;
  b_ = b;
  time_ms_ = time_ms;
  time_index_ = 0;
  direction_ = 0;

  auto [h, s, v] = RgbToHsv(r, g, b);
  h_ = h;
  s_ = s;
  v_ = v;
}

uint32_t LedController::Tick() {
  constexpr uint32_t kAlpha = 0xFF000000;

  if (mode_ == 0) {
    // Off
    return kAlpha;
  }

  if (mode_ == 1) {
    // Solid on
    return kAlpha | (static_cast<uint32_t>(r_) << 16) |
           (static_cast<uint32_t>(g_) << 8) | static_cast<uint32_t>(b_);
  }

  if (mode_ == 2) {
    // Blink
    time_index_++;
    if ((time_index_ * 10) > time_ms_) {
      time_index_ = 0;
      direction_ = direction_ ? 0 : 1;
    }
    if (direction_ == 0) {
      return kAlpha | (static_cast<uint32_t>(r_) << 16) |
             (static_cast<uint32_t>(g_) << 8) | static_cast<uint32_t>(b_);
    }
    return kAlpha;
  }

  if (mode_ == 3) {
    // Dimming (HSV brightness sweep)
    double step = (v_ * 100.0) / (time_ms_ / 10.0);
    if (direction_ == 0) {
      time_index_ += static_cast<int16_t>(step);
    } else {
      time_index_ -= static_cast<int16_t>(step);
    }

    int16_t max_index = static_cast<int16_t>(v_ * 100.0);
    if (time_index_ < 0) {
      time_index_ = 0;
      direction_ = 0;
    } else if (time_index_ > max_index) {
      time_index_ = max_index;
      direction_ = 1;
    }

    auto [r, g, b] = HsvToRgb(h_, s_, time_index_ / 100.0);
    return kAlpha | (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
  }

  return kAlpha;
}

std::tuple<double, double, double> LedController::RgbToHsv(uint8_t r,
                                                           uint8_t g,
                                                           uint8_t b) {
  double rn = r / 255.0;
  double gn = g / 255.0;
  double bn = b / 255.0;

  double max_val = std::max({rn, gn, bn});
  double min_val = std::min({rn, gn, bn});
  double delta = max_val - min_val;

  double h = 0.0;
  if (delta != 0.0) {
    if (max_val == rn) {
      h = 60.0 * std::fmod((gn - bn) / delta, 6.0);
    } else if (max_val == gn) {
      h = 60.0 * (((bn - rn) / delta) + 2.0);
    } else {
      h = 60.0 * (((rn - gn) / delta) + 4.0);
    }
  }
  if (h < 0.0) h += 360.0;

  double s = (max_val == 0.0) ? 0.0 : delta / max_val;
  double v = max_val;

  return {h, s, v};
}

std::tuple<uint8_t, uint8_t, uint8_t> LedController::HsvToRgb(double h,
                                                               double s,
                                                               double v) {
  double c = v * s;
  double x = c * (1.0 - std::fabs(std::fmod(h / 60.0, 2.0) - 1.0));
  double m = v - c;

  double rp, gp, bp;
  if (h < 60.0)       { rp = c; gp = x; bp = 0; }
  else if (h < 120.0) { rp = x; gp = c; bp = 0; }
  else if (h < 180.0) { rp = 0; gp = c; bp = x; }
  else if (h < 240.0) { rp = 0; gp = x; bp = c; }
  else if (h < 300.0) { rp = x; gp = 0; bp = c; }
  else                { rp = c; gp = 0; bp = x; }

  auto to_u8 = [](double val) -> uint8_t {
    return static_cast<uint8_t>(std::round(val * 255.0));
  };

  return {to_u8(rp + m), to_u8(gp + m), to_u8(bp + m)};
}

}  // namespace pinky
