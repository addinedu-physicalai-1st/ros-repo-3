#include "pinky_core/hal/ipc_camera.h"
#include <iostream>

namespace pinky {

IpcCamera::IpcCamera(const std::string& endpoint)
    : endpoint_(endpoint),
      ctx_(1),
      sub_sock_(ctx_, zmq::socket_type::sub) {}

IpcCamera::~IpcCamera() {
  if (initialized_) {
    sub_sock_.close();
  }
}

bool IpcCamera::Init() {
  try {
    sub_sock_.connect(endpoint_);
    sub_sock_.set(zmq::sockopt::subscribe, ""); // Subscribe to all
    // Set rcvtimeo to avoid blocking forever if python server is down
    sub_sock_.set(zmq::sockopt::rcvtimeo, 500); // 500ms timeout
    
    initialized_ = true;
    std::cout << "IpcCamera connected to " << endpoint_ << "\n";
    return true;
  } catch (const zmq::error_t& e) {
    std::cerr << "IpcCamera connect error: " << e.what() << "\n";
    return false;
  }
}

bool IpcCamera::CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                            uint16_t& width, uint16_t& height) {
  if (!initialized_) return false;

  zmq::message_t msg;
  auto res = sub_sock_.recv(msg, zmq::recv_flags::none);
  if (!res || msg.empty()) {
    return false;
  }

  jpeg_out.resize(msg.size());
  memcpy(jpeg_out.data(), msg.data(), msg.size());

  // Fixed resolution for now, or we could prefix the message with resolution
  width = 640;
  height = 480;

  return true;
}

}  // namespace pinky
