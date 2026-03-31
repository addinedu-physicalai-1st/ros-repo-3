#pragma once

#include <arpa/inet.h>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace pinky {

// Simple UDP sender/receiver.
class UdpServer {
 public:
  explicit UdpServer(uint16_t port);
  ~UdpServer();

  // Non-copyable
  UdpServer(const UdpServer&) = delete;
  UdpServer& operator=(const UdpServer&) = delete;

  // Initialize socket
  bool Start();
  void Stop();

  // Set the target address for outgoing packets
  void SetTargetEndpoint(const std::string& ip, uint16_t port);

  // Send data to the configured target endpoint
  bool Send(const std::vector<uint8_t>& data);

 private:
  uint16_t port_;
  int socket_fd_{-1};

  std::mutex target_mutex_;
  sockaddr_in target_addr_{};
  bool has_target_{false};
};

}  // namespace pinky
