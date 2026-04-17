#pragma once

#include <cstdint>
#include <vector>
#include <sys/types.h>

#include "pinky_core/hal/interfaces.h"

namespace pinky {

// Captures camera frames by spawning rpicam-vid as a child process and
// reading the raw MJPEG stream from its stdout pipe.  No Python or ZMQ
// needed — works entirely in C++ on Raspberry Pi 5 (ov5647 / imx219).
class RpicamCapture : public ICameraDriver {
 public:
  struct Config {
    int width{640};
    int height{480};
    int fps{15};
    int quality{85};         // MJPEG quality (1-100)
    bool rotate_180{true};   // maps to --hflip --vflip
  };

  RpicamCapture();
  explicit RpicamCapture(const Config& cfg);
  ~RpicamCapture() override;

  bool Init() override;
  bool CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                   uint16_t& width, uint16_t& height) override;

 private:
  Config cfg_;
  pid_t child_pid_{0};
  int pipe_fd_{-1};
  std::vector<uint8_t> buf_;

  static constexpr size_t kMaxBufBytes{1u << 20};  // 1 MB hard cap
};

}  // namespace pinky
