#pragma once

#include "pinky_core/hal/interfaces.h"
#include <vector>

namespace pinky {

class Ws2811Led : public ILedDriver {
 public:
  struct Config {
    int pin{18};       // GPIO
    int num_leds{8};   // Number of LEDs
    int strip_type{0}; // Usually WS2811_STRIP_GRB
  };

  explicit Ws2811Led(const Config& config);
  ~Ws2811Led() override;

  bool Init() override;
  void SetPixel(int index, uint8_t r, uint8_t g, uint8_t b) override;
  void Show() override;
  void Clear() override;

 private:
  Config config_;
  // Minimal representation for ws2811 internal struct pointer
  void* ws2811_{nullptr}; 
};

// Reusing same logic for Lamp but diff pin.
class Ws2811Lamp : public Ws2811Led {
 public:
  explicit Ws2811Lamp(const Ws2811Led::Config& config) : Ws2811Led(config) {}
};

}  // namespace pinky
