#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pinky {

// Emotion IDs
enum class EmotionId : uint8_t {
  kNeutral = 0,
  kHappy = 1,
  kSad = 2,
  kAngry = 3,
  kSurprised = 4,
  kSleepy = 5,
};

struct GifFrame {
  std::vector<uint8_t> pixels; // RGB565, width * height * 2
  int delay_ms;
};

struct AnimatedEmotion {
  std::vector<GifFrame> frames;
};

// Renders a static shape-based RGB565 emotion bitmap for the LCD.
std::vector<uint8_t> RenderEmotion(EmotionId emotion, int width = 320, int height = 240);

// Load an animated GIF (or static PNG/JPG) and resize it.
// Returns an AnimatedEmotion containing all frames.
AnimatedEmotion LoadAnimatedEmotion(const std::string& filepath,
                                    int width = 320, int height = 240);

}  // namespace pinky
