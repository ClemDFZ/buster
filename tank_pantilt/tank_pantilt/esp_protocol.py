"""Pan/tilt binary protocol: COBS + 0x00 delimiter + CRC-16/CCITT-FALSE."""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

PT_MAGIC = 0xA5
PT_VER = 0x03

PT_MODE_IDLE = 0
PT_MODE_HOMING = 1
PT_MODE_RUNNING = 2

PT_OP_VEL = 0x01
PT_OP_ANG = 0x02
PT_OP_DXY = 0x03
PT_OP_PAN = 0x04
PT_OP_TILT = 0x05
PT_OP_MODE = 0x06
PT_OP_STOP = 0x07
PT_OP_HOME = 0x08
PT_OP_STATUS_REQ = 0x09
PT_OP_STATUS = 0x81
PT_OP_MODE_EVT = 0x82
PT_OP_READY = 0x83

PT_CMD_SIZE = 18
PT_CMD_CRC_LEN = 16
PT_STATUS_SIZE = 26
PT_STATUS_CRC_LEN = 24

PACK_CMD = "<BBBBIffH"
PACK_STATUS = "<BBBBIffffH"

MODE_NAMES = {
    PT_MODE_IDLE: "IDLE",
    PT_MODE_HOMING: "HOMING",
    PT_MODE_RUNNING: "RUNNING",
}
MODE_BY_NAME = {v: k for k, v in MODE_NAMES.items()}

DEFAULT_BAUD = 921600


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    read = 0
    code_idx = 0
    code = 1
    n = len(data)
    out.append(0)
    while read < n:
        b = data[read]
        read += 1
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
            continue
        out.append(b)
        code += 1
        if code == 0xFF:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            break
        end = i + (code - 1)
        if end > n:
            raise ValueError("cobs overrun")
        out.extend(data[i:end])
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def _pack_cmd(op: int, mode: int, seq: int, f0: float, f1: float) -> bytes:
    body = struct.pack("<BBBBIff", PT_MAGIC, PT_VER, op, mode, seq, f0, f1)
    crc = crc16_ccitt_false(body)
    return body + struct.pack("<H", crc)


def pack_wire(frame: bytes) -> bytes:
    return cobs_encode(frame) + b"\x00"


def unpack_cmd(raw: bytes) -> Optional[tuple[int, int, int, float, float]]:
    if len(raw) != PT_CMD_SIZE:
        return None
    magic, ver, op, mode, seq, f0, f1, crc = struct.unpack(PACK_CMD, raw)
    if magic != PT_MAGIC or ver != PT_VER:
        return None
    if crc16_ccitt_false(raw[:PT_CMD_CRC_LEN]) != crc:
        return None
    return op, mode, seq, f0, f1


@dataclass
class PtStatus:
    mode: int
    seq: int
    pan_deg: float
    tilt_deg: float
    target_pan_deg: float
    target_tilt_deg: float

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, "UNKNOWN")


def unpack_status(raw: bytes) -> Optional[PtStatus]:
    if len(raw) != PT_STATUS_SIZE:
        return None
    vals = struct.unpack(PACK_STATUS, raw)
    magic, ver, op, mode, seq = vals[:5]
    if magic != PT_MAGIC or ver != PT_VER:
        return None
    if op not in (PT_OP_STATUS, PT_OP_MODE_EVT, PT_OP_READY):
        return None
    crc = vals[-1]
    if crc16_ccitt_false(raw[:PT_STATUS_CRC_LEN]) != crc:
        return None
    return PtStatus(
        mode=mode,
        seq=seq,
        pan_deg=vals[5],
        tilt_deg=vals[6],
        target_pan_deg=vals[7],
        target_tilt_deg=vals[8],
    )


def parse_frame(raw: bytes):
    if len(raw) == PT_CMD_SIZE:
        return unpack_cmd(raw)
    if len(raw) == PT_STATUS_SIZE:
        return unpack_status(raw)
    return None


def feed_cobs(buf: bytearray, chunk: bytes):
    """Split incoming serial bytes on 0x00, yield decoded raw frames."""
    buf.extend(chunk)
    while True:
        idx = buf.find(b"\x00")
        if idx < 0:
            break
        block = bytes(buf[:idx])
        del buf[: idx + 1]
        if not block:
            continue
        try:
            yield cobs_decode(block)
        except ValueError:
            continue


def cmd_vel(seq: int, pan_dps: float, tilt_dps: float) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_VEL, 0, seq, pan_dps, tilt_dps))


def cmd_ang(seq: int, pan_deg: float, tilt_deg: float) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_ANG, 0, seq, pan_deg, tilt_deg))


def cmd_dxy(seq: int, dpan: float, dtilt: float) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_DXY, 0, seq, dpan, dtilt))


def cmd_pan(seq: int, pan_deg: float) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_PAN, 0, seq, pan_deg, 0.0))


def cmd_tilt(seq: int, tilt_deg: float) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_TILT, 0, seq, 0.0, tilt_deg))


def cmd_mode(seq: int, mode_name: str) -> bytes:
    mode = MODE_BY_NAME.get(mode_name.upper(), PT_MODE_IDLE)
    return pack_wire(_pack_cmd(PT_OP_MODE, mode, seq, 0.0, 0.0))


def cmd_stop(seq: int) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_STOP, 0, seq, 0.0, 0.0))


def cmd_home(seq: int) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_HOME, 0, seq, 0.0, 0.0))


def cmd_status_req(seq: int) -> bytes:
    return pack_wire(_pack_cmd(PT_OP_STATUS_REQ, 0, seq, 0.0, 0.0))
