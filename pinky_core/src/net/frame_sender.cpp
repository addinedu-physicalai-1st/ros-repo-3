#include "pinky_core/net/frame_sender.h"

#include <opencv2/opencv.hpp>
#include "pinky_core/common/types.h"

namespace pinky {

FrameSender::FrameSender(std::shared_ptr<TcpServer> tcp, std::shared_ptr<Serializer> serializer)
    : tcp_(std::move(tcp)), serializer_(std::move(serializer)) {}

bool FrameSender::BroadcastFrame(const cv::Mat& bgr_image, int jpeg_quality) {
  if (bgr_image.empty() || !tcp_->IsRunning()) {
    return false;
  }

  // Set the compression parameters
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(jpeg_quality);

  CameraFrame frame;
  frame.width = static_cast<uint16_t>(bgr_image.cols);
  frame.height = static_cast<uint16_t>(bgr_image.rows);

  // Compress image to JPEG in-memory
  if (!cv::imencode(".jpg", bgr_image, frame.jpeg_data, compression_params)) {
    return false;
  }

  // Serialize the CameraFrame payload
  std::vector<uint8_t> payload = serializer_->SerializeCameraFrame(frame);
  
  // Wrap in protocol header
  std::vector<uint8_t> complete_msg = serializer_->Frame(MsgType::kCameraFrame, payload);

  // Broadcast to all active TCP clients
  tcp_->Broadcast(complete_msg);
  
  return true;
}

}  // namespace pinky
