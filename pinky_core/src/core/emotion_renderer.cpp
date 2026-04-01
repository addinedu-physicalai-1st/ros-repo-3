#include "pinky_core/core/emotion_renderer.h"

#include <cmath>
#include <cstring>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

namespace pinky {

namespace {

constexpr int kWidth = 240;
constexpr int kHeight = 240;
constexpr int kBufSize = kWidth * kHeight * 2;  // RGB565

// Pack R,G,B (0-255) into RGB565 big-endian (ILI9341 native byte order)
uint16_t Rgb565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  return (c >> 8) | (c << 8);  // swap bytes for SPI
}

void SetPixel(uint8_t* buf, int x, int y, uint16_t color) {
  if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
  int idx = (y * kWidth + x) * 2;
  std::memcpy(&buf[idx], &color, 2);
}

void FillCircle(uint8_t* buf, int cx, int cy, int r, uint16_t color) {
  for (int y = cy - r; y <= cy + r; ++y) {
    for (int x = cx - r; x <= cx + r; ++x) {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy <= r * r) {
        SetPixel(buf, x, y, color);
      }
    }
  }
}

void FillEllipse(uint8_t* buf, int cx, int cy, int rx, int ry, uint16_t color) {
  for (int y = cy - ry; y <= cy + ry; ++y) {
    for (int x = cx - rx; x <= cx + rx; ++x) {
      double dx = static_cast<double>(x - cx) / rx;
      double dy = static_cast<double>(y - cy) / ry;
      if (dx * dx + dy * dy <= 1.0) {
        SetPixel(buf, x, y, color);
      }
    }
  }
}

void FillRect(uint8_t* buf, int x0, int y0, int w, int h, uint16_t color) {
  for (int y = y0; y < y0 + h; ++y) {
    for (int x = x0; x < x0 + w; ++x) {
      SetPixel(buf, x, y, color);
    }
  }
}

void DrawArc(uint8_t* buf, int cx, int cy, int r, float start_deg,
             float end_deg, int thickness, uint16_t color) {
  constexpr float kDeg2Rad = 3.14159265f / 180.0f;
  for (float deg = start_deg; deg <= end_deg; deg += 0.5f) {
    float rad = deg * kDeg2Rad;
    for (int t = 0; t < thickness; ++t) {
      int x = cx + static_cast<int>((r - t) * std::cos(rad));
      int y = cy + static_cast<int>((r - t) * std::sin(rad));
      SetPixel(buf, x, y, color);
    }
  }
}

void Fill(uint8_t* buf, uint16_t color) {
  for (int i = 0; i < kWidth * kHeight; ++i) {
    std::memcpy(&buf[i * 2], &color, 2);
  }
}

// Function to load a gif/image and center it on black background
bool LoadImageIntoBuffer(const char* filepath, uint8_t* out_buf) {
  int w, h, channels;
  unsigned char *img = stbi_load(filepath, &w, &h, &channels, 3); // force RGB
  if (!img) {
    std::cerr << "Failed to load image: " << filepath << "\n";
    return false;
  }
  
  // Center image
  int offset_x = (kWidth - w) / 2;
  int offset_y = (kHeight - h) / 2;
  
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int px = x + offset_x;
      int py = y + offset_y;
      if (px >= 0 && px < kWidth && py >= 0 && py < kHeight) {
        int img_idx = (y * w + x) * 3;
        uint8_t r = img[img_idx];
        uint8_t g = img[img_idx + 1];
        uint8_t b = img[img_idx + 2];
        SetPixel(out_buf, px, py, Rgb565(r, g, b));
      }
    }
  }
  stbi_image_free(img);
  return true;
}

}  // namespace

std::vector<uint8_t> RenderEmotion(EmotionId emotion) {
  std::vector<uint8_t> buf(kBufSize, 0);
  uint16_t bg = Rgb565(0, 0, 0); // Black background to save power
  uint16_t white = Rgb565(255, 255, 255);
  uint16_t black = Rgb565(0, 0, 0);
  uint16_t pink = Rgb565(255, 100, 150);

  Fill(buf.data(), bg);

  switch (emotion) {
    case EmotionId::kNeutral:
      // Try to load basic.gif as requested
      if (!LoadImageIntoBuffer("../../pinky_pro/src/pinky_pro/pinky_emotion/emotion/basic.gif", buf.data())) {
          // Fallback if file not found
          FillCircle(buf.data(), 80, 100, 20, white);
          FillCircle(buf.data(), 160, 100, 20, white);
          FillCircle(buf.data(), 80, 100, 8, black);
          FillCircle(buf.data(), 160, 100, 8, black);
          FillRect(buf.data(), 85, 170, 70, 5, white);
      }
      break;

    case EmotionId::kHappy:
      if (!LoadImageIntoBuffer("../../pinky_pro/src/pinky_pro/pinky_emotion/emotion/happy.gif", buf.data())) {
          DrawArc(buf.data(), 80, 110, 20, 200, 340, 4, white);
          DrawArc(buf.data(), 160, 110, 20, 200, 340, 4, white);
          DrawArc(buf.data(), 120, 155, 35, 20, 160, 4, white);
          FillCircle(buf.data(), 50, 145, 12, pink);
          FillCircle(buf.data(), 190, 145, 12, pink);
      }
      break;

    case EmotionId::kSad:
      if (!LoadImageIntoBuffer("../../pinky_pro/src/pinky_pro/pinky_emotion/emotion/sad.gif", buf.data())) {
          FillCircle(buf.data(), 80, 100, 22, white);
          FillCircle(buf.data(), 160, 100, 22, white);
          FillCircle(buf.data(), 80, 105, 8, black);
          FillCircle(buf.data(), 160, 105, 8, black);
          DrawArc(buf.data(), 120, 195, 30, 200, 340, 4, white);
      }
      break;

    case EmotionId::kAngry:
      if (!LoadImageIntoBuffer("../../pinky_pro/src/pinky_pro/pinky_emotion/emotion/angry.gif", buf.data())) {
          FillEllipse(buf.data(), 80, 100, 22, 14, white);
          FillEllipse(buf.data(), 160, 100, 22, 14, white);
          FillCircle(buf.data(), 80, 100, 7, black);
          FillCircle(buf.data(), 160, 100, 7, black);
          FillRect(buf.data(), 60, 72, 45, 4, white);
          FillRect(buf.data(), 135, 72, 45, 4, white);
          FillRect(buf.data(), 85, 170, 70, 6, white);
      }
      break;

    case EmotionId::kSurprised:
      // No surprised gif, fallback to shape
      FillCircle(buf.data(), 80, 100, 28, white);
      FillCircle(buf.data(), 160, 100, 28, white);
      FillCircle(buf.data(), 80, 100, 12, black);
      FillCircle(buf.data(), 160, 100, 12, black);
      FillCircle(buf.data(), 120, 175, 18, white);
      FillCircle(buf.data(), 120, 175, 10, bg);
      break;

    case EmotionId::kSleepy:
      if (!LoadImageIntoBuffer("../../pinky_pro/src/pinky_pro/pinky_emotion/emotion/bored.gif", buf.data())) {
          FillRect(buf.data(), 58, 100, 44, 4, white);
          FillRect(buf.data(), 138, 100, 44, 4, white);
          FillRect(buf.data(), 175, 55, 20, 3, white);
          FillRect(buf.data(), 185, 45, 15, 3, white);
          FillRect(buf.data(), 195, 35, 10, 3, white);
          FillRect(buf.data(), 100, 170, 40, 4, white);
      }
      break;
  }

  return buf;
}

}  // namespace pinky
