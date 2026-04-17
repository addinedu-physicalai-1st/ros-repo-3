#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pinky_core/hal/interfaces.h"

namespace pinky {

class Ili9341Lcd : public ILcdDriver {
 public:
  struct Config {
    int spi_channel{0};
    int spi_speed{80000000}; // 80MHz
    int rst_pin{27};
    int dc_pin{25};
    int bl_pin{18};          // Backlight
    int width{320};
    int height{240};
  };

  explicit Ili9341Lcd(const Config& config);
  ~Ili9341Lcd() override;

  bool Init() override;

  void DrawFrame(const uint8_t* buffer, size_t size) override;
  void DrawFrameRgb565(const uint8_t* buffer, size_t size) override;
  void ClearScreen(uint16_t color_rgb565 = 0x0000) override;
  void DrawText(int x, int y, const std::string& text,
                uint16_t fg = 0xFFFF, uint16_t bg = 0x0000) override;
  int Width() const override { return config_.width; }
  int Height() const override { return config_.height; }

 private:
  void SpiWriteCommand(uint8_t cmd);
  void SpiWriteData(uint8_t data);
  void SpiWriteDataBlock(const uint8_t* data, size_t len);
  void SetWindow(int x0, int y0, int x1, int y1);
  void DrawChar(int x, int y, char c, uint16_t fg, uint16_t bg);

  Config config_;
  int spi_fd_{-1};
  std::vector<uint8_t> rgb565_buffer_;
};

}  // namespace pinky
