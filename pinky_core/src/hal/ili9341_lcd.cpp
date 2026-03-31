#include "pinky_core/hal/ili9341_lcd.h"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "wiringPi.h"
#include "wiringPiSPI.h"

namespace pinky {

Ili9341Lcd::Ili9341Lcd(const Config& config) : config_(config) {
  rgb565_buffer_.resize(config_.width * config_.height * 2);
}

Ili9341Lcd::~Ili9341Lcd() {
  if (spi_fd_ != -1) {
    // wiringPiSPI does not have explicit close, standard close works
    close(spi_fd_);
  }
}

bool Ili9341Lcd::Init() {
  if (wiringPiSetupGpio() < 0) {
    std::cerr << "Ili9341Lcd: Failed to setup wiringPi GPIO\n";
    return false;
  }

  pinMode(config_.dc_pin, OUTPUT);
  pinMode(config_.rst_pin, OUTPUT);
  pinMode(config_.bl_pin, OUTPUT);

  digitalWrite(config_.bl_pin, HIGH); // Backlight ON

  spi_fd_ = wiringPiSPISetup(config_.spi_channel, config_.spi_speed);
  if (spi_fd_ < 0) {
    std::cerr << "Ili9341Lcd: Failed to setup SPI\n";
    return false;
  }

  // Hardware Reset
  digitalWrite(config_.rst_pin, HIGH);
  delay(10);
  digitalWrite(config_.rst_pin, LOW);
  delay(10);
  digitalWrite(config_.rst_pin, HIGH);
  delay(120);

  // Magic initialization sequence for ILI9341
  SpiWriteCommand(0x01); // Software Reset
  delay(150);
  
  SpiWriteCommand(0x11); // Sleep Out
  delay(120);

  SpiWriteCommand(0x3A); // Pixel Format Set
  SpiWriteData(0x55);    // 16-bit RGB 5-6-5 format

  SpiWriteCommand(0x36); // Memory Access Control
  SpiWriteData(0x28);    // Landscape

  SpiWriteCommand(0x29); // Display ON
  // For actual ILI9341 there are more gamma/power settings, 
  // keeping minimal required sequence for standard modules.

  std::cout << "ILI9341 initialized on SPI channel " << config_.spi_channel << "\n";
  return true;
}

void Ili9341Lcd::SpiWriteCommand(uint8_t cmd) {
  digitalWrite(config_.dc_pin, LOW); // command mode
  wiringPiSPIDataRW(config_.spi_channel, &cmd, 1);
}

void Ili9341Lcd::SpiWriteData(uint8_t data) {
  digitalWrite(config_.dc_pin, HIGH); // data mode
  wiringPiSPIDataRW(config_.spi_channel, &data, 1);
}

void Ili9341Lcd::SpiWriteDataBlock(const uint8_t* data, size_t len) {
  digitalWrite(config_.dc_pin, HIGH); // data mode
  
  // wiringPiSPIDataRW overwrites buffer, we must send chunks if we don't want to destroy original buf,
  // but since our rgb565_buffer is already what we're sending, we can write directly or chunk it 
  // if limits exceed. Linux spidev typically maxes at 4096 bytes without configuration.
  
  constexpr size_t kChunkSize = 4096;
  size_t offset = 0;
  
  while (offset < len) {
    size_t chunk = std::min(kChunkSize, len - offset);
    // Destructive to rgb565_buffer_, but we redraw completely each frame so it's fine.
    wiringPiSPIDataRW(config_.spi_channel, const_cast<uint8_t*>(data + offset), chunk);
    offset += chunk;
  }
}

void Ili9341Lcd::DrawFrame(const uint8_t* buffer, size_t size) {
  // Assume buffer is RGB888, size = W*H*3
  size_t expected_size = config_.width * config_.height * 3;
  if (size != expected_size) {
    return;
  }

  // Convert RGB888 to RGB565
  for (size_t i = 0, j = 0; i < size; i += 3, j += 2) {
    uint8_t r = buffer[i];
    uint8_t g = buffer[i + 1];
    uint8_t b = buffer[i + 2];

    uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    
    // ILI9341 expects big-endian 16-bit values over SPI
    rgb565_buffer_[j] = (color >> 8) & 0xFF;
    rgb565_buffer_[j + 1] = color & 0xFF;
  }

  // Set window (column/page)
  SpiWriteCommand(0x2A); // CASET
  SpiWriteData(0x00);
  SpiWriteData(0x00);
  SpiWriteData((config_.width - 1) >> 8);
  SpiWriteData((config_.width - 1) & 0xFF);

  SpiWriteCommand(0x2B); // PASET
  SpiWriteData(0x00);
  SpiWriteData(0x00);
  SpiWriteData((config_.height - 1) >> 8);
  SpiWriteData((config_.height - 1) & 0xFF);

  SpiWriteCommand(0x2C); // RAMWR

  SpiWriteDataBlock(rgb565_buffer_.data(), rgb565_buffer_.size());
}

}  // namespace pinky
