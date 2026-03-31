#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "pinky_core/common/types.h"

namespace pinky {

class IMotorDriver {
 public:
  virtual ~IMotorDriver() = default;
  virtual bool Init() = 0;
  virtual void SetVelocityWait(double left_rad_s, double right_rad_s) = 0;
  virtual JointState ReadJointState() = 0;
};

class ILidarDriver {
 public:
  virtual ~ILidarDriver() = default;
  virtual bool Init() = 0;
  virtual bool StartScan() = 0;
  virtual void StopScan() = 0;
  virtual bool GetScan(LidarScan& scan) = 0;
};

class IImuDriver {
 public:
  virtual ~IImuDriver() = default;
  virtual bool Init() = 0;
  virtual bool ReadData(ImuData& data) = 0;
};

class IAdcDriver {
 public:
  virtual ~IAdcDriver() = default;
  virtual bool Init() = 0;
  virtual bool ReadAll(uint16_t& ch0, uint16_t& ch1, uint16_t& ch2, uint16_t& ch3, uint16_t& ch4) = 0;
};

class ILedDriver {
 public:
  virtual ~ILedDriver() = default;
  virtual bool Init() = 0;
  virtual void SetPixel(int index, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void Show() = 0;
  virtual void Clear() = 0;
};

class ILcdDriver {
 public:
  virtual ~ILcdDriver() = default;
  virtual bool Init() = 0;
  
  // Frame buffer is expected to be RGB565 or RGB888 depending on implementation
  virtual void DrawFrame(const uint8_t* buffer, size_t size) = 0;

  // Add text and clear screen utilities for core app to display status
  virtual void ClearScreen(uint16_t color_rgb565 = 0x0000) = 0;
  virtual void DrawText(int x, int y, const std::string& text, uint16_t fg = 0xFFFF, uint16_t bg = 0x0000) = 0;
};

}  // namespace pinky
