#include "pinky_core/hal/rpicam_capture.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

namespace pinky {

RpicamCapture::RpicamCapture() : cfg_(Config{}) {}

RpicamCapture::RpicamCapture(const Config& cfg) : cfg_(cfg) {}

RpicamCapture::~RpicamCapture() {
  if (child_pid_ > 0) {
    kill(child_pid_, SIGTERM);
    int status;
    waitpid(child_pid_, &status, 0);
    child_pid_ = 0;
  }
  if (pipe_fd_ >= 0) {
    close(pipe_fd_);
    pipe_fd_ = -1;
  }
}

bool RpicamCapture::Init() {
  int pipefd[2];
  if (pipe(pipefd) != 0) {
    std::cerr << "RpicamCapture: pipe() failed: " << strerror(errno) << "\n";
    return false;
  }

  child_pid_ = fork();
  if (child_pid_ < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    std::cerr << "RpicamCapture: fork() failed\n";
    return false;
  }

  if (child_pid_ == 0) {
    // ── Child process ──────────────────────────────────────────────
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[1]);

    char ws[16], hs[16], fs[16], qs[16];
    snprintf(ws, sizeof(ws), "%d", cfg_.width);
    snprintf(hs, sizeof(hs), "%d", cfg_.height);
    snprintf(fs, sizeof(fs), "%d", cfg_.fps);
    snprintf(qs, sizeof(qs), "%d", cfg_.quality);

    // Build argv dynamically — no manual index counting
    std::vector<char*> args;
    args.push_back(const_cast<char*>("rpicam-vid"));
    args.push_back(const_cast<char*>("--width"));     args.push_back(ws);
    args.push_back(const_cast<char*>("--height"));    args.push_back(hs);
    args.push_back(const_cast<char*>("--framerate")); args.push_back(fs);
    args.push_back(const_cast<char*>("--codec"));     args.push_back(const_cast<char*>("mjpeg"));
    args.push_back(const_cast<char*>("--quality"));   args.push_back(qs);
    args.push_back(const_cast<char*>("--output"));    args.push_back(const_cast<char*>("-"));
    args.push_back(const_cast<char*>("--nopreview"));
    args.push_back(const_cast<char*>("--timeout"));   args.push_back(const_cast<char*>("0"));
    if (cfg_.rotate_180) {
      args.push_back(const_cast<char*>("--hflip"));
      args.push_back(const_cast<char*>("--vflip"));
    }
    args.push_back(nullptr);

    execvp("rpicam-vid", args.data());
    args[0] = const_cast<char*>("libcamera-vid");
    execvp("libcamera-vid", args.data());

    std::cerr << "RpicamCapture: exec failed — rpicam-vid/libcamera-vid not found\n";
    _exit(1);
  }

  // ── Parent process ────────────────────────────────────────────────
  close(pipefd[1]);
  pipe_fd_ = pipefd[0];

  usleep(1'500'000);

  int status;
  pid_t result = waitpid(child_pid_, &status, WNOHANG);
  if (result == child_pid_) {
    std::cerr << "RpicamCapture: rpicam-vid exited immediately "
              << "(exit code " << WEXITSTATUS(status) << ")\n";
    close(pipe_fd_);
    pipe_fd_ = -1;
    child_pid_ = 0;
    return false;
  }

  std::cout << "RpicamCapture: started rpicam-vid (PID " << child_pid_
            << "), " << cfg_.width << "x" << cfg_.height
            << " @ " << cfg_.fps << " fps, quality " << cfg_.quality << "\n";
  return true;
}

bool RpicamCapture::CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                                 uint16_t& width, uint16_t& height) {
  if (pipe_fd_ < 0) return false;

  constexpr size_t kChunkSize = 65536;
  uint8_t chunk[kChunkSize];

  // ── Phase 1: drain all immediately available data from pipe ────────
  bool got_data = false;
  while (true) {
    struct pollfd pfd{pipe_fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, 0);  // non-blocking
    if (ret <= 0) break;
    if (pfd.revents & POLLHUP) {
      close(pipe_fd_); pipe_fd_ = -1; return false;
    }
    ssize_t n = read(pipe_fd_, chunk, kChunkSize);
    if (n <= 0) { close(pipe_fd_); pipe_fd_ = -1; return false; }
    buf_.insert(buf_.end(), chunk, chunk + n);
    got_data = true;
  }

  // If no data was immediately available, block up to 500ms for some
  if (!got_data && buf_.empty()) {
    struct pollfd pfd{pipe_fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, 500);
    if (ret <= 0) return false;
    if (pfd.revents & POLLHUP) { close(pipe_fd_); pipe_fd_ = -1; return false; }
    ssize_t n = read(pipe_fd_, chunk, kChunkSize);
    if (n <= 0) { close(pipe_fd_); pipe_fd_ = -1; return false; }
    buf_.insert(buf_.end(), chunk, chunk + n);
  }

  if (buf_.size() > kMaxBufBytes) { buf_.clear(); return false; }

  // ── Phase 2: find the LAST complete JPEG (latest frame) ───────────
  // Scan backwards for the last EOI marker (FF D9)
  ssize_t last_eoi = -1;
  for (ssize_t i = static_cast<ssize_t>(buf_.size()) - 1; i >= 1; --i) {
    if (buf_[i - 1] == 0xFF && buf_[i] == 0xD9) {
      last_eoi = i + 1;
      break;
    }
  }
  if (last_eoi < 0) return false;  // no complete frame yet

  // Scan backwards from EOI to find the matching SOI (FF D8)
  ssize_t last_soi = -1;
  for (ssize_t i = last_eoi - 2; i >= 1; --i) {
    if (buf_[i - 1] == 0xFF && buf_[i] == 0xD8) {
      last_soi = i - 1;
      break;
    }
  }
  if (last_soi < 0) {
    buf_.erase(buf_.begin(), buf_.begin() + last_eoi);
    return false;
  }

  // Extract the latest frame and discard everything before it
  jpeg_out.assign(buf_.begin() + last_soi, buf_.begin() + last_eoi);
  buf_.erase(buf_.begin(), buf_.begin() + last_eoi);

  width  = static_cast<uint16_t>(cfg_.width);
  height = static_cast<uint16_t>(cfg_.height);
  return true;
}

}  // namespace pinky
