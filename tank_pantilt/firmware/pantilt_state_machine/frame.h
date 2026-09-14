#pragma once
#include <stddef.h>
#include <stdint.h>

#define PT_MAGIC 0xA5
#define PT_VER 0x03

#define PT_MODE_IDLE 0
#define PT_MODE_HOMING 1
#define PT_MODE_RUNNING 2

enum PtOp : uint8_t {
  PT_OP_VEL = 0x01,
  PT_OP_ANG = 0x02,
  PT_OP_DXY = 0x03,
  PT_OP_PAN = 0x04,
  PT_OP_TILT = 0x05,
  PT_OP_MODE = 0x06,
  PT_OP_STOP = 0x07,
  PT_OP_HOME = 0x08,
  PT_OP_STATUS_REQ = 0x09,
  PT_OP_STATUS = 0x81,
  PT_OP_MODE_EVT = 0x82,
  PT_OP_READY = 0x83,
};

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t ver;
  uint8_t op;
  uint8_t mode;
  uint32_t seq;
  float f0;
  float f1;
  uint16_t crc16;
} PtCmdFrame;

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t ver;
  uint8_t op;
  uint8_t mode;
  uint32_t seq;
  float pan_deg;
  float tilt_deg;
  float target_pan_deg;
  float target_tilt_deg;
  uint16_t crc16;
} PtStatusFrame;

static const size_t PT_CMD_SIZE = sizeof(PtCmdFrame);
static const size_t PT_CMD_CRC_LEN = offsetof(PtCmdFrame, crc16);
static const size_t PT_STATUS_SIZE = sizeof(PtStatusFrame);
static const size_t PT_STATUS_CRC_LEN = offsetof(PtStatusFrame, crc16);
