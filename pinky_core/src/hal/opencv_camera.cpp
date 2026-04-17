#include "pinky_core/hal/opencv_camera.h"
#include <iostream>

namespace pinky {

OpencvCamera::OpencvCamera() {}

OpencvCamera::~OpencvCamera() {
#ifdef PINKY_HAS_OPENCV
  if (cap_.isOpened()) {
    cap_.release();
  }
#endif
}

bool OpencvCamera::Init() {
#ifdef PINKY_HAS_OPENCV
  std::cout << "OpencvCamera: Attempting direct V4L2 capture on /dev/video0...\n";
  
  // Directly open V4L2 device 0 without GStreamer to prevent Pi 5 libcamera crashes
  cap_.open(0, cv::CAP_V4L2);
  
  if (!cap_.isOpened()) {
    std::cout << "OpencvCamera: V4L2 failed, trying default API...\n";
    cap_.open(0);
  }
  
  if (!cap_.isOpened()) {
    std::cerr << "OpencvCamera: Failed to open camera device 0\n";
    return false;
  }
  
  // Set lower resolution for stream efficiency
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap_.set(cv::CAP_PROP_FPS, 15);
  
  // Optional: V4L2 pixel format
  // cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

  std::cout << "OpencvCamera: Initialized successfully (Backend: " << cap_.getBackendName() << ").\n";
  return true;
#else
  std::cerr << "OpencvCamera: OpenCV not available, camera disabled.\n";
  return false;
#endif
}

bool OpencvCamera::CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                               uint16_t& width, uint16_t& height) {
#ifdef PINKY_HAS_OPENCV
  if (!cap_.isOpened()) return false;

  cv::Mat frame;
  if (!cap_.read(frame) || frame.empty()) {
    return false;
  }

  // Rotate 180 degrees to match hardware orientation
  cv::rotate(frame, frame, cv::ROTATE_180);

  width = static_cast<uint16_t>(frame.cols);
  height = static_cast<uint16_t>(frame.rows);

  std::vector<int> params;
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(60);

  return cv::imencode(".jpg", frame, jpeg_out, params);
#else
  return false;
#endif
}

}  // namespace pinky
