#pragma once

#include <cstdint>
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

// Renders a 240x240 RGB565 emotion bitmap for ILI9341 LCD.
// Returns a pixel buffer of 240*240*2 bytes.
std::vector<uint8_t> RenderEmotion(EmotionId emotion);

}  // namespace pinky
