#include "pinky_core/net/connection_manager.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>

namespace pinky {

ConnectionManager::ConnectionManager(std::shared_ptr<TcpServer> tcp, std::shared_ptr<UdpServer> udp)
    : tcp_(std::move(tcp)), udp_(std::move(udp)) {}

ConnectionManager::~ConnectionManager() {
  Stop();
}

void ConnectionManager::Start() {
  if (running_.load()) return;
  running_.store(true);
  watcher_thread_ = std::thread(&ConnectionManager::WatcherLoop, this);
}

void ConnectionManager::Stop() {
  if (!running_.exchange(false)) return;
  if (watcher_thread_.joinable()) watcher_thread_.join();
}

void ConnectionManager::OnClientConnected(int fd, const std::string& ip_addr) {
  std::cout << "ConnectionManager: Client " << fd << " connected from " << ip_addr << "\n";
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (current_active_fd_ == -1) {
    current_active_fd_ = fd;
    current_active_ip_ = ip_addr;
    if (udp_) udp_->SetTargetEndpoint(ip_addr, 9200);
  }

  sessions_[fd] = {ip_addr, std::chrono::steady_clock::now()};
}

void ConnectionManager::OnClientDisconnected(int fd) {
  std::cout << "ConnectionManager: Client " << fd << " disconnected\n";
  std::lock_guard<std::mutex> lock(mutex_);
  
  sessions_.erase(fd);
  
  if (current_active_fd_ == fd) {
    current_active_fd_ = -1;
    current_active_ip_.clear();
    
    // Switch to another active if exists
    if (!sessions_.empty()) {
      auto it = sessions_.begin();
      current_active_fd_ = it->first;
      current_active_ip_ = it->second.ip;
      if (udp_) udp_->SetTargetEndpoint(current_active_ip_, 9200);
    } else {
      if (udp_) udp_->SetTargetEndpoint("", 0);
    }
  }
}

void ConnectionManager::OnPingReceived(int fd, uint64_t client_ts) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sessions_.find(fd);
  if (it != sessions_.end()) {
    it->second.last_ping = std::chrono::steady_clock::now();
  }
}

bool ConnectionManager::GetActiveClient(int& fd, std::string& ip) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_active_fd_ == -1) return false;
  fd = current_active_fd_;
  ip = current_active_ip_;
  return true;
}

void ConnectionManager::WatcherLoop() {
  constexpr auto kPingTimeout = std::chrono::seconds(5);
  
  while (running_.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    auto now = std::chrono::steady_clock::now();
    std::vector<int> timeouts;
    
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& [fd, session] : sessions_) {
        if (now - session.last_ping > kPingTimeout) {
          timeouts.push_back(fd);
        }
      }
    }
    
    for (int fd : timeouts) {
      std::cerr << "ConnectionManager: Client " << fd << " ping timeout. Disconnecting.\n";
      // Close the socket at TcpServer level; this triggers the connection callback
      // which calls OnClientDisconnected to clean up our session state.
      if (tcp_) tcp_->ForceDisconnect(fd);
    }
  }
}

}  // namespace pinky
