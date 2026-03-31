#pragma once

#include <memory>
#include <vector>

#include "pinky_core/net/tcp_server.h"
#include "pinky_core/protocol/serializer.h"

// Forward declaration if cv::Mat is used to avoid heavy OpenCV include here
namespace cv {
class Mat;
}

namespace pinky {

class FrameSender {
 public:
  FrameSender(std::shared_ptr<TcpServer> tcp, std::shared_ptr<Serializer> serializer);
  
  // Compresses the BGR image to JPEG and broadcasts to all connected TCP clients.
  bool BroadcastFrame(const cv::Mat& bgr_image, int jpeg_quality = 60);

 private:
  std::shared_ptr<TcpServer> tcp_;
  std::shared_ptr<Serializer> serializer_;
};

}  // namespace pinky
