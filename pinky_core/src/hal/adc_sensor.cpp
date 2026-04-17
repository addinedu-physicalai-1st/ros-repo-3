#include "pinky_core/hal/adc_sensor.h"

#include <iostream>
#include <unistd.h>

#include "wiringPiI2C.h"
#include "wiringPi.h"

namespace pinky {

AdcSensor::AdcSensor(const Config& config) : config_(config) {}

AdcSensor::~AdcSensor() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool AdcSensor::Init() {
  fd_ = wiringPiI2CSetupInterface(config_.interface.c_str(), config_.address);
  if (fd_ == -1) {
    std::cerr << "AdcSensor: Failed to init I2C communication on " << config_.interface << "\n";
    return false;
  }
  return true;
}

bool AdcSensor::ReadAll(uint16_t& ch0, uint16_t& ch1, uint16_t& ch2, uint16_t& ch3, uint16_t& ch4) {
  if (fd_ == -1) return false;

  uint8_t registers[5] = {0x88, 0xC8, 0x98, 0xD8, 0xF8};
  uint16_t adc_result[5] = {0};

  for(int i = 0; i < 5; i++) {
    uint8_t data[2] = {0};

    // Use raw write/read due to custom ADC timing behavior
    wiringPiI2CRawWrite(fd_, &registers[i], 1);
    delayMicroseconds(6000); // 6ms delay

    if (wiringPiI2CRawRead(fd_, data, 2) < 0) {
      return false; // I2C error
    }

    adc_result[i] = (static_cast<uint16_t>(data[0]) << 4) | (static_cast<uint16_t>(data[1]) >> 4);
  }

  ch0 = adc_result[0];
  ch1 = adc_result[1];
  ch2 = adc_result[2];
  ch3 = adc_result[3];
  ch4 = adc_result[4];

  return true;
}

}  // namespace pinky
