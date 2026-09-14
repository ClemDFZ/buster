#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, MSB-first, no XOROUT. */
static inline uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

#ifdef __cplusplus
}
#endif
