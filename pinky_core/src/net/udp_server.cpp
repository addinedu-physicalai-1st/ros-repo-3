#include "pinky_core/net/udp_server.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace pinky {

UdpServer::UdpServer(uint16_t port) : port_(port) {}

UdpServer::~UdpServer() {
  Stop();
}

bool UdpServer::Start() {
  if (socket_fd_ >= 0) return false;

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "UdpServer: failed to create socket\n";
    return false;
  }

  // Non-blocking
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  int opt = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

  // Bind to allow receiving if needed
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "UdpServer: bind failed\n";
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  std::cout << "UdpServer started on port " << port_ << "\n";
  return true;
}

void UdpServer::Stop() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void UdpServer::SetTargetEndpoint(const std::string& ip, uint16_t port) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_addr_.sin_family = AF_INET;
  target_addr_.sin_port = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &target_addr_.sin_addr) <= 0) {
    std::cerr << "UdpServer: Invalid IP address: " << ip << "\n";
    has_target_ = false;
  } else {
    has_target_ = true;
  }
}

bool UdpServer::Send(const std::vector<uint8_t>& data) {
  if (socket_fd_ < 0 || data.empty()) return false;

  sockaddr_in target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!has_target_) return false;
    target = target_addr_;
  }

  ssize_t sent = sendto(socket_fd_, data.data(), data.size(), 0,
                        reinterpret_cast<struct sockaddr*>(&target), sizeof(target));
  if (sent < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    std::cerr << "UdpServer: sendto failed: " << strerror(errno) << "\n";
    return false;
  }
  return true;
}

}  // namespace pinky
