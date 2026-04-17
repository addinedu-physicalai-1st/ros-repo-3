#pragma once

#include "pinky_core/hal/interfaces.h"

#ifdef PINKY_HAS_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace pinky {

class OpencvCamera : public ICameraDriver {
 public:
  OpencvCamera();
  ~OpencvCamera() override;

  bool Init() override;
  bool CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                   uint16_t& width, uint16_t& height) override;

 private:
#ifdef PINKY_HAS_OPENCV
  cv::VideoCapture cap_;
#endif
};

}  // namespace pinky