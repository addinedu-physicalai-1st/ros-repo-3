#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <zmq.hpp>
#include "pinky_core/hal/interfaces.h"

namespace pinky {

class IpcCamera : public ICameraDriver {
 public:
  IpcCamera(const std::string& endpoint = "tcp://127.0.0.1:5555");
  virtual ~IpcCamera();

  virtual bool Init() override;
  virtual bool CaptureJpeg(std::vector<uint8_t>& jpeg_out, uint16_t& width, uint16_t& height) override;

 private:
  std::string endpoint_;
  zmq::context_t ctx_;
  zmq::socket_t sub_sock_;
  bool initialized_ = false;
};

}  // namespace pinky
