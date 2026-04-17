#include "pinky_core/hal/ws2811_led.h"

#include <iostream>

// Reusing rpi_ws281x lib APIs
// Usually included via "ws2811.h"
extern "C" {
// Opaque forward declaration for library types
typedef struct ws2811_channel_t {
    int gpionum;
    int invert;
    int count;
    int strip_type;
    uint32_t *leds;
    uint8_t brightness;
    uint8_t wshift;
    uint8_t rshift;
    uint8_t gshift;
    uint8_t bshift;
    uint8_t gamma[256];
} ws2811_channel_t;

typedef struct ws2811_t {
    uint64_t render_wait_time;
    void *device;
    const void *rpi_hw;
    uint32_t freq;
    int dmanum;
    ws2811_channel_t channel[2];
} ws2811_t;

int ws2811_init(ws2811_t *ws2811);
void ws2811_fini(ws2811_t *ws2811);
int ws2811_render(ws2811_t *ws2811);
void ws2811_wait(ws2811_t *ws2811);
}

namespace pinky {

Ws2811Led::Ws2811Led(const Config& config) : config_(config) {
  // Allocate ws2811 structure memory
  ws2811_ = new ws2811_t();
}

Ws2811Led::~Ws2811Led() {
  if (ws2811_) {
    if (initialized_) {
      Clear();
      Show();
      ws2811_fini(static_cast<ws2811_t*>(ws2811_));
    }
    delete static_cast<ws2811_t*>(ws2811_);
    ws2811_ = nullptr;
  }
}

bool Ws2811Led::Init() {
  auto* ws = static_cast<ws2811_t*>(ws2811_);
  
  // Set target frequency, dma etc per library defaults
  ws->freq = 800000;
  ws->dmanum = 10;
  
  // Configure channel 0
  ws->channel[0].gpionum = config_.pin;
  ws->channel[0].count = config_.num_leds;
  ws->channel[0].invert = 0;
  ws->channel[0].brightness = 255;
  ws->channel[0].strip_type = config_.strip_type; // WS2811_STRIP_GRB
  
  // Disable channel 1
  ws->channel[1].gpionum = 0;
  ws->channel[1].count = 0;
  ws->channel[1].invert = 0;
  ws->channel[1].brightness = 0;

  int ret = ws2811_init(ws);
  if (ret != 0) { // WS2811_SUCCESS
    std::cerr << "Ws2811Led: Failed to init WS2811 on pin " << config_.pin << "\n";
    return false;
  }

  initialized_ = true;
  return true;
}

void Ws2811Led::SetPixel(int index, uint8_t r, uint8_t g, uint8_t b) {
  if (!initialized_) return;
  auto* ws = static_cast<ws2811_t*>(ws2811_);
  if (ws && index >= 0 && index < config_.num_leds) {
    // Pack color ARGB format
    uint32_t color = (static_cast<uint32_t>(r) << 16) | 
                     (static_cast<uint32_t>(g) << 8) | 
                     (static_cast<uint32_t>(b));
    ws->channel[0].leds[index] = color;
  }
}

void Ws2811Led::Show() {
  if (!initialized_) return;
  auto* ws = static_cast<ws2811_t*>(ws2811_);
  if (ws) {
    ws2811_render(ws);
  }
}

void Ws2811Led::Clear() {
  for (int i = 0; i < config_.num_leds; ++i) {
    SetPixel(i, 0, 0, 0);
  }
}

}  // namespace pinky
