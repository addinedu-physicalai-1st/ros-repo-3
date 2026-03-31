#include "pinky_core/net/tcp_server.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace pinky {

namespace {
constexpr int kMaxEvents = 64;
constexpr int kBufferSize = 4096;

bool SetNonBlocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1) return false;
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}
}  // namespace

TcpServer::TcpServer(uint16_t port) : port_(port) {}

TcpServer::~TcpServer() {
  Stop();
}

void TcpServer::SetMessageCallback(TcpMessageCallback cb) {
  on_message_ = std::move(cb);
}

void TcpServer::SetConnectionCallback(TcpConnectionCallback cb) {
  on_connection_ = std::move(cb);
}

bool TcpServer::Start() {
  if (running_.load()) return false;

  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::cerr << "TcpServer: failed to create socket\n";
    return false;
  }

  int opt = 1;
  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    std::cerr << "TcpServer: setsockopt failed\n";
    close(server_fd_);
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);

  if (bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "TcpServer: bind failed\n";
    close(server_fd_);
    return false;
  }

  if (listen(server_fd_, SOMAXCONN) < 0) {
    std::cerr << "TcpServer: listen failed\n";
    close(server_fd_);
    return false;
  }

  if (!SetNonBlocking(server_fd_)) {
    close(server_fd_);
    return false;
  }

  epoll_fd_ = epoll_create1(0);
  if (epoll_fd_ < 0) {
    std::cerr << "TcpServer: epoll_create1 failed\n";
    close(server_fd_);
    return false;
  }

  struct epoll_event ev{};
  ev.events = EPOLLIN | EPOLLET;
  ev.data.fd = server_fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, server_fd_, &ev) < 0) {
    std::cerr << "TcpServer: epoll_ctl ADD server_fd failed\n";
    close(server_fd_);
    close(epoll_fd_);
    return false;
  }

  running_.store(true);
  thread_ = std::thread(&TcpServer::RunLoop, this);

  return true;
}

void TcpServer::Stop() {
  if (!running_.exchange(false)) return;

  if (thread_.joinable()) {
    thread_.join();
  }

  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (const auto& [fd, state] : clients_) {
    if (on_connection_) on_connection_(fd, false, "");
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
    close(fd);
  }
  clients_.clear();

  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }
  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }
}

bool TcpServer::Send(int client_fd, const std::vector<uint8_t>& data) {
  if (data.empty()) return false;

  std::lock_guard<std::mutex> lock(clients_mutex_);
  if (clients_.find(client_fd) == clients_.end()) return false;

  size_t total_sent = 0;
  while (total_sent < data.size()) {
    ssize_t sent = send(client_fd, data.data() + total_sent, data.size() - total_sent, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(1000);
        continue;
      }
      return false;
    }
    total_sent += sent;
  }
  return true;
}

void TcpServer::Broadcast(const std::vector<uint8_t>& data) {
  if (data.empty()) return;

  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (const auto& [fd, state] : clients_) {
    size_t total_sent = 0;
    while (total_sent < data.size()) {
      ssize_t sent = send(fd, data.data() + total_sent, data.size() - total_sent, MSG_NOSIGNAL);
      if (sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          usleep(1000);
          continue;
        }
        break;
      }
      total_sent += sent;
    }
  }
}

void TcpServer::ForceDisconnect(int client_fd) {
  DisconnectClient(client_fd);
}

void TcpServer::RunLoop() {
  epoll_event events[kMaxEvents];

  while (running_.load()) {
    int num_events = epoll_wait(epoll_fd_, events, kMaxEvents, 100);
    if (num_events < 0) {
      if (errno == EINTR) continue;
      break;
    }

    for (int i = 0; i < num_events; ++i) {
      if (events[i].data.fd == server_fd_) {
        AcceptClient();
      } else {
        int client_fd = events[i].data.fd;
        if (events[i].events & (EPOLLERR | EPOLLHUP | EPOLLRDHUP)) {
          DisconnectClient(client_fd);
        } else if (events[i].events & EPOLLIN) {
          ReadClient(client_fd);
        }
      }
    }
  }
}

void TcpServer::AcceptClient() {
  while (true) {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int client_fd = accept(server_fd_, reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);
    
    if (client_fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      break;
    }

    SetNonBlocking(client_fd);

    struct epoll_event ev{};
    ev.events = EPOLLIN | EPOLLET | EPOLLRDHUP;
    ev.data.fd = client_fd;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, client_fd, &ev) < 0) {
      close(client_fd);
      continue;
    }

    char ip_buf[INET_ADDRSTRLEN]{};
    inet_ntop(AF_INET, &client_addr.sin_addr, ip_buf, sizeof(ip_buf));
    std::string client_ip(ip_buf);

    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients_[client_fd] = ClientState{{}, client_ip};
    }

    if (on_connection_) on_connection_(client_fd, true, client_ip);
  }
}

void TcpServer::ReadClient(int fd) {
  uint8_t buffer[kBufferSize];
  bool disconnected = false;

  std::vector<uint8_t>* client_buffer = nullptr;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = clients_.find(fd);
    if (it == clients_.end()) return;
    client_buffer = &it->second.recv_buffer;
  }

  while (true) {
    ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      disconnected = true;
      break;
    } else if (count == 0) {
      disconnected = true;
      break;
    }

    client_buffer->insert(client_buffer->end(), buffer, buffer + count);
    
    if (on_message_) {
      ParsedMessage msg;
      size_t bytes_consumed = 0;
      while (client_buffer->size() >= kHeaderSize) { // basic check
        ParseResult res = ParseMessage(client_buffer->data(), client_buffer->size(), msg, bytes_consumed);
        if (res == ParseResult::kOk) {
          on_message_(fd, msg);
          client_buffer->erase(client_buffer->begin(), client_buffer->begin() + bytes_consumed);
        } else if (res == ParseResult::kIncomplete) {
          break;
        } else {
          // Parsing error, drop a byte and try to re-sync
          client_buffer->erase(client_buffer->begin());
        }
      }
    }
  }

  if (disconnected) {
    DisconnectClient(fd);
  }
}

void TcpServer::DisconnectClient(int fd) {
  bool call_cb = false;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (clients_.find(fd) != clients_.end()) {
      clients_.erase(fd);
      call_cb = true;
    }
  }

  if (call_cb) {
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
    close(fd);
    if (on_connection_) on_connection_(fd, false, "");
  }
}

}  // namespace pinky
