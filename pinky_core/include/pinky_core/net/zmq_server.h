#pragma once

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <zmq.hpp>
#include "pinky.pb.h"

namespace pinky {

class ZmqServer {
 public:
  ZmqServer(uint16_t rep_port, uint16_t pub_port);
  ~ZmqServer();

  bool Start();
  void Stop();

  void PublishTelemetry(const proto::SensorTelemetry& telemetry);
  void PublishVideo(const proto::VideoStream& video);

  using CommandCallback = std::function<void(const proto::ControlCommand&, proto::CommandAck&)>;
  void SetCommandCallback(CommandCallback cb);

 private:
  void RepLoop();

  uint16_t rep_port_;
  uint16_t pub_port_;
  
  zmq::context_t ctx_;
  zmq::socket_t rep_sock_;
  zmq::socket_t pub_sock_;

  std::atomic<bool> running_{false};
  std::thread rep_thread_;
  CommandCallback cmd_callback_;
};

}  // namespace pinky
