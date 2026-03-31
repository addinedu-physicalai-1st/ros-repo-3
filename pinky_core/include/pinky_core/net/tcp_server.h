#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "pinky_core/protocol/message_types.h"
#include "pinky_core/protocol/serializer.h"

namespace pinky {

// Callback invoked when a complete message is received from a client.
// Parameters: client_fd, parsed message.
using TcpMessageCallback = std::function<void(int, const ParsedMessage&)>;

// Callback invoked when a client connects or disconnects.
// Parameters: fd, connected, client_ip (empty on disconnect).
using TcpConnectionCallback = std::function<void(int, bool, const std::string&)>;

// Single-threaded epoll-based TCP server.
// Call Start() to spawn the accept/read loop in a background thread.
class TcpServer {
 public:
  explicit TcpServer(uint16_t port);
  ~TcpServer();

  // Non-copyable
  TcpServer(const TcpServer&) = delete;
  TcpServer& operator=(const TcpServer&) = delete;

  void SetMessageCallback(TcpMessageCallback cb);
  void SetConnectionCallback(TcpConnectionCallback cb);

  bool Start();
  void Stop();

  // Send a pre-framed message to a specific client.
  bool Send(int client_fd, const std::vector<uint8_t>& data);

  // Send a pre-framed message to all connected clients.
  void Broadcast(const std::vector<uint8_t>& data);

  // Force-disconnect a client by fd (thread-safe, callable from outside).
  void ForceDisconnect(int client_fd);

  bool IsRunning() const { return running_.load(); }

 private:
  void RunLoop();
  void AcceptClient();
  void ReadClient(int fd);
  void DisconnectClient(int fd);

  uint16_t port_;
  int server_fd_{-1};
  int epoll_fd_{-1};

  std::atomic<bool> running_{false};
  std::thread thread_;

  TcpMessageCallback on_message_;
  TcpConnectionCallback on_connection_;

  // Per-client receive buffers
  struct ClientState {
    std::vector<uint8_t> recv_buffer;
    std::string ip;
  };
  std::mutex clients_mutex_;
  std::unordered_map<int, ClientState> clients_;
};

}  // namespace pinky
