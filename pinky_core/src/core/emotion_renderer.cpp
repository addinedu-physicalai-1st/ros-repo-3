#include "pinky_core/core/emotion_renderer.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#ifdef PINKY_HAS_OPENCV
#include <opencv2/opencv.hpp>
#endif

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

namespace pinky {

namespace {

// Pack R,G,B (0-255) into RGB565 big-endian (ILI9341 native byte order)
uint16_t Rgb565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  return static_cast<uint16_t>((c >> 8) | (c << 8));  // swap bytes for SPI
}

// All primitives take canvas dimensions as parameters
void SetPixel(uint8_t* buf, int canvas_w, int canvas_h,
              int x, int y, uint16_t color) {
  if (x < 0 || x >= canvas_w || y < 0 || y >= canvas_h) return;
  int idx = (y * canvas_w + x) * 2;
  std::memcpy(&buf[idx], &color, 2);
}

void FillCircle(uint8_t* buf, int cw, int ch,
                int cx, int cy, int r, uint16_t color) {
  for (int y = cy - r; y <= cy + r; ++y) {
    for (int x = cx - r; x <= cx + r; ++x) {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy <= r * r) {
        SetPixel(buf, cw, ch, x, y, color);
      }
    }
  }
}

void FillEllipse(uint8_t* buf, int cw, int ch,
                 int cx, int cy, int rx, int ry, uint16_t color) {
  for (int y = cy - ry; y <= cy + ry; ++y) {
    for (int x = cx - rx; x <= cx + rx; ++x) {
      double dx = static_cast<double>(x - cx) / rx;
      double dy = static_cast<double>(y - cy) / ry;
      if (dx * dx + dy * dy <= 1.0) {
        SetPixel(buf, cw, ch, x, y, color);
      }
    }
  }
}

void FillRect(uint8_t* buf, int cw, int ch,
              int x0, int y0, int w, int h, uint16_t color) {
  for (int y = y0; y < y0 + h; ++y) {
    for (int x = x0; x < x0 + w; ++x) {
      SetPixel(buf, cw, ch, x, y, color);
    }
  }
}

void DrawArc(uint8_t* buf, int cw, int ch,
             int cx, int cy, int r, float start_deg,
             float end_deg, int thickness, uint16_t color) {
  constexpr float kDeg2Rad = 3.14159265f / 180.0f;
  for (float deg = start_deg; deg <= end_deg; deg += 0.5f) {
    float rad = deg * kDeg2Rad;
    for (int t = 0; t < thickness; ++t) {
      int x = cx + static_cast<int>((r - t) * std::cos(rad));
      int y = cy + static_cast<int>((r - t) * std::sin(rad));
      SetPixel(buf, cw, ch, x, y, color);
    }
  }
}

void Fill(uint8_t* buf, int canvas_w, int canvas_h, uint16_t color) {
  int total = canvas_w * canvas_h;
  for (int i = 0; i < total; ++i) {
    std::memcpy(&buf[i * 2], &color, 2);
  }
}

// Load first frame of an image (GIF/PNG/JPG), resize to target dims,
// render onto RGB565 buffer with black background for transparency.
bool LoadAndResizeImage(const std::string& filepath, uint8_t* out_buf,
                        int target_w, int target_h) {
  int src_w = 0;
  int src_h = 0;
  int channels = 0;
  unsigned char* img = stbi_load(filepath.c_str(), &src_w, &src_h, &channels, 4);
  if (!img) {
    return false;
  }

  // Nearest-neighbor resize with alpha-aware rendering
  for (int ty = 0; ty < target_h; ++ty) {
    int sy = ty * src_h / target_h;
    for (int tx = 0; tx < target_w; ++tx) {
      int sx = tx * src_w / target_w;
      int src_idx = (sy * src_w + sx) * 4;
      uint8_t r = img[src_idx + 0];
      uint8_t g = img[src_idx + 1];
      uint8_t b = img[src_idx + 2];
      uint8_t a = img[src_idx + 3];

      // Skip fully transparent pixels (keep black background)
      if (a < 32) continue;

      // Alpha-blend against black background
      if (a < 224) {
        r = static_cast<uint8_t>(r * a / 255);
        g = static_cast<uint8_t>(g * a / 255);
        b = static_cast<uint8_t>(b * a / 255);
      }

      uint16_t color = Rgb565(r, g, b);
      int out_idx = (ty * target_w + tx) * 2;
      std::memcpy(&out_buf[out_idx], &color, 2);
    }
  }

  stbi_image_free(img);
  return true;
}

}  // namespace

std::vector<uint8_t> RenderEmotion(EmotionId emotion, int width, int height) {
  int buf_size = width * height * 2;  // RGB565
  std::vector<uint8_t> buf(buf_size, 0);
  uint16_t bg = Rgb565(0, 0, 0);
  uint16_t white = Rgb565(255, 255, 255);
  uint16_t black = Rgb565(0, 0, 0);
  uint16_t pink = Rgb565(255, 100, 150);

  Fill(buf.data(), width, height, bg);

  // Face center offsets (centered on 320x240 canvas)
  int cx = width / 2;   // 160
  int cy = height / 2;  // 120

  switch (emotion) {
    case EmotionId::kNeutral:
      // Eyes
      FillCircle(buf.data(), width, height, cx - 40, cy - 20, 20, white);
      FillCircle(buf.data(), width, height, cx + 40, cy - 20, 20, white);
      FillCircle(buf.data(), width, height, cx - 40, cy - 20, 8, black);
      FillCircle(buf.data(), width, height, cx + 40, cy - 20, 8, black);
      // Mouth
      FillRect(buf.data(), width, height, cx - 35, cy + 50, 70, 5, white);
      break;

    case EmotionId::kHappy:
      // Eyes: arcs
      DrawArc(buf.data(), width, height, cx - 40, cy - 10, 20, 200, 340, 4, white);
      DrawArc(buf.data(), width, height, cx + 40, cy - 10, 20, 200, 340, 4, white);
      // Smile
      DrawArc(buf.data(), width, height, cx, cy + 35, 35, 20, 160, 4, white);
      // Cheeks
      FillCircle(buf.data(), width, height, cx - 70, cy + 25, 12, pink);
      FillCircle(buf.data(), width, height, cx + 70, cy + 25, 12, pink);
      break;

    case EmotionId::kSad:
      FillCircle(buf.data(), width, height, cx - 40, cy - 20, 22, white);
      FillCircle(buf.data(), width, height, cx + 40, cy - 20, 22, white);
      FillCircle(buf.data(), width, height, cx - 40, cy - 15, 8, black);
      FillCircle(buf.data(), width, height, cx + 40, cy - 15, 8, black);
      // Frown
      DrawArc(buf.data(), width, height, cx, cy + 75, 30, 200, 340, 4, white);
      break;

    case EmotionId::kAngry:
      FillEllipse(buf.data(), width, height, cx - 40, cy - 20, 22, 14, white);
      FillEllipse(buf.data(), width, height, cx + 40, cy - 20, 22, 14, white);
      FillCircle(buf.data(), width, height, cx - 40, cy - 20, 7, black);
      FillCircle(buf.data(), width, height, cx + 40, cy - 20, 7, black);
      // Eyebrows
      FillRect(buf.data(), width, height, cx - 60, cy - 48, 45, 4, white);
      FillRect(buf.data(), width, height, cx + 15, cy - 48, 45, 4, white);
      // Mouth
      FillRect(buf.data(), width, height, cx - 35, cy + 50, 70, 6, white);
      break;

    case EmotionId::kSurprised:
      FillCircle(buf.data(), width, height, cx - 40, cy - 20, 28, white);
      FillCircle(buf.data(), width, height, cx + 40, cy - 20, 28, white);
      FillCircle(buf.data(), width, height, cx - 40, cy - 20, 12, black);
      FillCircle(buf.data(), width, height, cx + 40, cy - 20, 12, black);
      // O mouth
      FillCircle(buf.data(), width, height, cx, cy + 55, 18, white);
      FillCircle(buf.data(), width, height, cx, cy + 55, 10, bg);
      break;

    case EmotionId::kSleepy:
      // Closed eyes
      FillRect(buf.data(), width, height, cx - 62, cy - 20, 44, 4, white);
      FillRect(buf.data(), width, height, cx + 18, cy - 20, 44, 4, white);
      // ZZZ
      FillRect(buf.data(), width, height, cx + 55, cy - 65, 20, 3, white);
      FillRect(buf.data(), width, height, cx + 65, cy - 75, 15, 3, white);
      FillRect(buf.data(), width, height, cx + 75, cy - 85, 10, 3, white);
      // Small mouth
      FillRect(buf.data(), width, height, cx - 20, cy + 50, 40, 4, white);
      break;
  }

  return buf;
}

AnimatedEmotion LoadAnimatedEmotion(const std::string& filepath, int target_w, int target_h) {
  AnimatedEmotion result;

  std::ifstream file(filepath, std::ios::binary | std::ios::ate);
  if (!file.is_open()) return result;
  
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buffer(size);
  if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) return result;

  int src_w = 0, src_h = 0, frames = 0, comp = 0;
  int* delays = nullptr;
  
  unsigned char* img = stbi_load_gif_from_memory(buffer.data(), static_cast<int>(size),
                                                 &delays, &src_w, &src_h, &frames, &comp, 4);
  
  // Fallback to static image if it's not an animated GIF
  if (!img) {
    img = stbi_load_from_memory(buffer.data(), static_cast<int>(size),
                                &src_w, &src_h, &comp, 4);
    if (!img) return result;
    frames = 1;
  }

  for (int f = 0; f < frames; ++f) {
    GifFrame frame;
    frame.pixels.resize(target_w * target_h * 2, 0);
    frame.delay_ms = delays ? delays[f] : 100; // Default 100ms if no delay
    
    // Scale from STB delay (which is often in ms)
    if (frame.delay_ms < 20) frame.delay_ms = 100;

    unsigned char* src_frame = img + (f * src_w * src_h * 4);

    // Nearest-neighbor resize with alpha-aware rendering
    for (int ty = 0; ty < target_h; ++ty) {
      int sy = ty * src_h / target_h;
      for (int tx = 0; tx < target_w; ++tx) {
        int sx = tx * src_w / target_w;
        int src_idx = (sy * src_w + sx) * 4;
        uint8_t r = src_frame[src_idx + 0];
        uint8_t g = src_frame[src_idx + 1];
        uint8_t b = src_frame[src_idx + 2];
        uint8_t a = src_frame[src_idx + 3];

        // Skip fully transparent pixels (keep black background)
        if (a < 32) continue;

        // Alpha-blend against black background
        if (a < 224) {
          r = static_cast<uint8_t>(r * a / 255);
          g = static_cast<uint8_t>(g * a / 255);
          b = static_cast<uint8_t>(b * a / 255);
        }

        uint16_t color = Rgb565(r, g, b);
        int out_idx = (ty * target_w + tx) * 2;
        std::memcpy(&frame.pixels[out_idx], &color, 2);
      }
    }
    result.frames.push_back(std::move(frame));
  }

  stbi_image_free(img);
  if (delays) stbi_image_free(delays);
  
  return result;
}

}  // namespace pinky
