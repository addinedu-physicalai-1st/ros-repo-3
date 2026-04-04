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
    // NOTE: stderr is intentionally NOT redirected so rpicam-vid errors are
    // visible in the parent's terminal for debugging.

    char ws[16], hs[16], fs[16];
    snprintf(ws, sizeof(ws), "%d", cfg_.width);
    snprintf(hs, sizeof(hs), "%d", cfg_.height);
    snprintf(fs, sizeof(fs), "%d", cfg_.fps);

    char* argv_base[] = {
      const_cast<char*>("rpicam-vid"),
      const_cast<char*>("--width"),     ws,
      const_cast<char*>("--height"),    hs,
      const_cast<char*>("--framerate"), fs,
      const_cast<char*>("--codec"),     const_cast<char*>("mjpeg"),
      const_cast<char*>("--output"),    const_cast<char*>("-"),
      const_cast<char*>("--nopreview"),
      const_cast<char*>("--timeout"),   const_cast<char*>("0"),
      nullptr, nullptr, nullptr  // placeholders for hflip/vflip/sentinel
    };
    constexpr int kBaseArgc = 13;

    if (cfg_.rotate_180) {
      argv_base[kBaseArgc]     = const_cast<char*>("--hflip");
      argv_base[kBaseArgc + 1] = const_cast<char*>("--vflip");
      argv_base[kBaseArgc + 2] = nullptr;
    }

    execvp("rpicam-vid", argv_base);
    argv_base[0] = const_cast<char*>("libcamera-vid");
    execvp("libcamera-vid", argv_base);

    // Both exec attempts failed — stderr is visible here
    std::cerr << "RpicamCapture: exec failed — rpicam-vid/libcamera-vid not found in PATH\n";
    _exit(1);
  }

  // ── Parent process ────────────────────────────────────────────────────────
  close(pipefd[1]);
  pipe_fd_ = pipefd[0];

  // Give rpicam-vid time to initialise
  usleep(1'500'000);  // 1.5s — Pi 5 ISP init can take >800ms

  // Check if child already exited (exec failure or camera error)
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
            << " @ " << cfg_.fps << " fps\n";
  return true;
}

bool RpicamCapture::CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                                 uint16_t& width, uint16_t& height) {
  if (pipe_fd_ < 0) return false;

  constexpr size_t kChunkSize = 8192;
  uint8_t chunk[kChunkSize];

  while (true) {
    struct pollfd pfd{pipe_fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, 500);
    if (ret < 0) return false;   // poll error
    if (ret == 0) return false;  // timeout — no data from rpicam-vid

    // POLLHUP: write end closed (rpicam-vid exited)
    if (pfd.revents & POLLHUP) {
      std::cerr << "RpicamCapture: rpicam-vid pipe closed (process exited)\n";
      close(pipe_fd_);
      pipe_fd_ = -1;
      return false;
    }

    ssize_t n = read(pipe_fd_, chunk, kChunkSize);
    if (n <= 0) {
      // EOF: child process exited
      close(pipe_fd_);
      pipe_fd_ = -1;
      return false;
    }

    buf_.insert(buf_.end(), chunk, chunk + n);

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
      uint8_t last = buf_.back();
      buf_.clear();
      buf_.push_back(last);
      continue;
    }

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
    // SOI found but no EOI yet — keep reading
  }
}

}  // namespace pinky
