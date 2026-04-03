#include "pinky_core/net/zmq_server.h"
#include <iostream>

namespace pinky {

ZmqServer::ZmqServer(uint16_t rep_port, uint16_t pub_port)
    : rep_port_(rep_port),
      pub_port_(pub_port),
      ctx_(1),
      rep_sock_(ctx_, zmq::socket_type::rep),
      pub_sock_(ctx_, zmq::socket_type::pub) {}

ZmqServer::~ZmqServer() {
  Stop();
}

bool ZmqServer::Start() {
  try {
    std::string rep_addr = "tcp://*:" + std::to_string(rep_port_);
    rep_sock_.bind(rep_addr);

    std::string pub_addr = "tcp://*:" + std::to_string(pub_port_);
    pub_sock_.bind(pub_addr);

    running_ = true;
    rep_thread_ = std::thread(&ZmqServer::RepLoop, this);
    return true;
  } catch (const zmq::error_t& e) {
    std::cerr << "ZmqServer bind error: " << e.what() << "\n";
    return false;
  }
}

void ZmqServer::Stop() {
  if (running_.exchange(false)) {
    // A trick to wake up the blocking recv in rep_thread_
    try {
      zmq::socket_t waker(ctx_, zmq::socket_type::req);
      waker.connect("tcp://127.0.0.1:" + std::to_string(rep_port_));
      zmq::message_t req(0);
      waker.send(req, zmq::send_flags::none);
    } catch (...) {}

    if (rep_thread_.joinable()) {
      rep_thread_.join();
    }
  }
}

void ZmqServer::PublishTelemetry(const proto::SensorTelemetry& telemetry) {
  if (!running_) return;
  std::string buf;
  if (telemetry.SerializeToString(&buf)) {
    zmq::message_t topic("T", 1);
    pub_sock_.send(topic, zmq::send_flags::sndmore);

    zmq::message_t msg(buf.size());
    memcpy(msg.data(), buf.data(), buf.size());
    pub_sock_.send(msg, zmq::send_flags::none);
  }
}

void ZmqServer::PublishVideo(const proto::VideoStream& video) {
  if (!running_) return;
  std::string buf;
  if (video.SerializeToString(&buf)) {
    zmq::message_t topic("V", 1);
    pub_sock_.send(topic, zmq::send_flags::sndmore);

    zmq::message_t msg(buf.size());
    memcpy(msg.data(), buf.data(), buf.size());
    pub_sock_.send(msg, zmq::send_flags::none);
  }
}

void ZmqServer::SetCommandCallback(CommandCallback cb) {
  cmd_callback_ = cb;
}

void ZmqServer::RepLoop() {
  while (running_) {
    zmq::message_t req_msg;
    // Block until message arrives or context is terminated/socket closed
    auto res = rep_sock_.recv(req_msg, zmq::recv_flags::none);
    if (!res) continue;

    if (!running_) {
      // Sent by Stop() to wake up the thread
      zmq::message_t ack_msg(0);
      rep_sock_.send(ack_msg, zmq::send_flags::none);
      break;
    }

    proto::ControlCommand cmd;
    proto::CommandAck ack;
    if (cmd.ParseFromArray(req_msg.data(), req_msg.size())) {
      if (cmd_callback_) {
        cmd_callback_(cmd, ack);
      } else {
        ack.set_success(false);
        ack.set_message("No callback registered");
      }
    } else {
      ack.set_success(false);
      ack.set_message("Parse error");
    }

    std::string ack_buf;
    ack.SerializeToString(&ack_buf);
    zmq::message_t rep_msg(ack_buf.size());
    memcpy(rep_msg.data(), ack_buf.data(), ack_buf.size());
    rep_sock_.send(rep_msg, zmq::send_flags::none);
  }
}

}  // namespace pinky
