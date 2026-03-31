"""CRC16-CCITT — must produce identical output to C++ Crc16()."""


def crc16(data: bytes) -> int:
    """Compute CRC16-CCITT (polynomial 0x1021, initial 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
