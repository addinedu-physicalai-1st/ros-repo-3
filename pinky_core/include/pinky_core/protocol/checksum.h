#pragma once

#include <cstddef>
#include <cstdint>

namespace pinky {

// CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF)
uint16_t Crc16(const uint8_t* data, size_t length);

}  // namespace pinky
