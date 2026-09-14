#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max);
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max);

#ifdef __cplusplus
}
#endif
