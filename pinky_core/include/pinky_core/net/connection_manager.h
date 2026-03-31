#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "pinky_core/net/tcp_server.h"
#include "pinky_core/net/udp_server.h"

namespace pinky {

class ConnectionManager {
 public:
  ConnectionManager(std::shared_ptr<TcpServer> tcp, std::shared_ptr<UdpServer> udp);
  ~ConnectionManager();

  // Start ping timeout watcher thread
  void Start();
  void Stop();

  // Called from TcpServer
  void OnClientConnected(int fd, const std::string& ip_addr);
  void OnClientDisconnected(int fd);

  // Called when a PING is received from client
  void OnPingReceived(int fd, uint64_t client_ts);

  // Retrieves the current active client for UDP streaming
  // Returns false if no client is fully connected
  bool GetActiveClient(int& fd, std::string& ip);

 private:
  void WatcherLoop();

  std::shared_ptr<TcpServer> tcp_;
  std::shared_ptr<UdpServer> udp_;

  struct ClientSession {
    std::string ip;
    std::chrono::steady_clock::time_point last_ping;
  };
  std::mutex mutex_;
  std::map<int, ClientSession> sessions_;

  int current_active_fd_{-1};
  std::string current_active_ip_;

  std::atomic<bool> running_{false};
  std::thread watcher_thread_;
};

}  // namespace pinky
