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

// Renders an RGB565 emotion bitmap for the LCD.
// Returns a pixel buffer of width*height*2 bytes.
std::vector<uint8_t> RenderEmotion(EmotionId emotion, int width = 320, int height = 240);

// Load an image file (GIF/PNG/JPG first frame), resize to target dims,
// return as RGB565 buffer. Returns empty vector on failure.
std::vector<uint8_t> LoadEmotionImage(const std::string& filepath,
                                      int width = 320, int height = 240);

}  // namespace pinky
