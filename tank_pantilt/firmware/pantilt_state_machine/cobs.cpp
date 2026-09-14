#include "cobs.h"

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
  if (out_max == 0) return 0;

  size_t read_idx = 0;
  size_t write_idx = 1;
  size_t code_idx = 0;
  uint8_t code = 1;

  while (read_idx < len) {
    const uint8_t b = in[read_idx++];
    if (b == 0) {
      out[code_idx] = code;
      code = 1;
      code_idx = write_idx;
      if (write_idx >= out_max) return 0;
      out[write_idx++] = 0;
      continue;
    }

    if (write_idx >= out_max) return 0;
    out[write_idx++] = b;
    code++;

    if (code == 0xFF) {
      out[code_idx] = code;
      code = 1;
      code_idx = write_idx;
      if (write_idx >= out_max) return 0;
      out[write_idx++] = 0;
    }
  }

  out[code_idx] = code;
  return write_idx;
}

size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
  size_t read_idx = 0;
  size_t write_idx = 0;

  while (read_idx < len) {
    const uint8_t code = in[read_idx++];
    if (code == 0) break;

    const size_t end = read_idx + (size_t)(code - 1);
    if (end > len) return 0;

    while (read_idx < end) {
      if (write_idx >= out_max) return 0;
      out[write_idx++] = in[read_idx++];
    }

    if (code < 0xFF && read_idx < len) {
      if (write_idx >= out_max) return 0;
      out[write_idx++] = 0;
    }
  }

  return write_idx;
}
