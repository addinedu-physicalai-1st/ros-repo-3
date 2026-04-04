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
    // ── Child process ──────────────────────────────────────────────────────
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[1]);

    // Silence rpicam-vid diagnostic output
    int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      dup2(devnull, STDERR_FILENO);
      close(devnull);
    }

    // Build numeric string args (safe to do before exec)
    char ws[16], hs[16], fs[16];
    snprintf(ws, sizeof(ws), "%d", cfg_.width);
    snprintf(hs, sizeof(hs), "%d", cfg_.height);
    snprintf(fs, sizeof(fs), "%d", cfg_.fps);

    // argv without rotation
    char* argv_base[] = {
      const_cast<char*>("rpicam-vid"),
      const_cast<char*>("--width"),  ws,
      const_cast<char*>("--height"), hs,
      const_cast<char*>("--framerate"), fs,
      const_cast<char*>("--codec"),  const_cast<char*>("mjpeg"),
      const_cast<char*>("--output"), const_cast<char*>("-"),
      const_cast<char*>("--nopreview"),
      const_cast<char*>("--timeout"), const_cast<char*>("0"),
      nullptr, nullptr, nullptr  // placeholders for hflip/vflip/sentinel
    };
    int base_argc = 13; // index of first placeholder

    if (cfg_.rotate_180) {
      argv_base[base_argc]     = const_cast<char*>("--hflip");
      argv_base[base_argc + 1] = const_cast<char*>("--vflip");
      argv_base[base_argc + 2] = nullptr;
    }

    // Try rpicam-vid first, then libcamera-vid as fallback
    execvp("rpicam-vid", argv_base);
    argv_base[0] = const_cast<char*>("libcamera-vid");
    execvp("libcamera-vid", argv_base);

    std::cerr << "RpicamCapture: exec failed — rpicam-vid not found\n";
    _exit(1);
  }

  // ── Parent process ────────────────────────────────────────────────────────
  close(pipefd[1]);
  pipe_fd_ = pipefd[0];

  // Give rpicam-vid ~800 ms to initialise and start emitting frames
  usleep(800'000);

  std::cout << "RpicamCapture: started rpicam-vid (PID " << child_pid_
            << "), " << cfg_.width << "x" << cfg_.height
            << " @ " << cfg_.fps << " fps\n";
  return true;
}

bool RpicamCapture::CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                                 uint16_t& width, uint16_t& height) {
  if (pipe_fd_ < 0) return false;

  constexpr size_t kChunkSize = 8192;
  uint8_t chunk[kChunkSize];

  // Keep reading until we extract one complete JPEG frame.
  while (true) {
    // Wait up to 500 ms for data
    struct pollfd pfd{pipe_fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, 500);
    if (ret <= 0) return false;  // timeout or error

    ssize_t n = read(pipe_fd_, chunk, kChunkSize);
    if (n <= 0) return false;

    buf_.insert(buf_.end(), chunk, chunk + n);

    // Prevent runaway accumulation
    if (buf_.size() > kMaxBufBytes) {
      buf_.clear();
      continue;
    }

    // ── Find JPEG SOI marker (FF D8) ────────────────────────────────────────
    size_t soi = buf_.size();
    for (size_t i = 0; i + 1 < buf_.size(); ++i) {
      if (buf_[i] == 0xFF && buf_[i + 1] == 0xD8) {
        soi = i;
        break;
      }
    }

    if (soi == buf_.size()) {
      // No SOI yet — keep the last byte in case it straddles chunks
      uint8_t last = buf_.back();
      buf_.clear();
      buf_.push_back(last);
      continue;
    }

    // Discard bytes before SOI
    if (soi > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + soi);
      soi = 0;
    }

    // ── Find JPEG EOI marker (FF D9) after SOI ──────────────────────────────
    for (size_t i = 2; i + 1 < buf_.size(); ++i) {
      if (buf_[i] == 0xFF && buf_[i + 1] == 0xD9) {
        const size_t eoi = i + 2;
        jpeg_out.assign(buf_.begin(), buf_.begin() + eoi);
        buf_.erase(buf_.begin(), buf_.begin() + eoi);
        width  = static_cast<uint16_t>(cfg_.width);
        height = static_cast<uint16_t>(cfg_.height);
        return true;
      }
    }
    // SOI found but no EOI yet — continue reading
  }
}

}  // namespace pinky
