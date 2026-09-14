"""Binary DriveFrame v2 protocol (COBS + 0x00 delimiter). Compatible with odom_fusion."""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

IMU_FRAME_MAGIC = 0xA5
DRIVE_FRAME_VER = 0x02
DRIVE_FRAME_SIZE = 102
DRIVE_FRAME_CRC_LEN = 100
PACK_LE_V2 = "<BBHIQffffffffff" "ffff" "fff" "iiii" "H"


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


@dataclass
class DriveFrame:
    magic: int
    ver: int
    length: int
    seq: int
    t_us: int
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    wheel_omega: tuple[float, float, float, float]
    v_x_enc: float
    v_y_enc: float
    omega_z_enc: float
    tachy_total: tuple[int, int, int, int]
    crc16: int

    @staticmethod
    def unpack(raw: bytes) -> "DriveFrame":
        if len(raw) != DRIVE_FRAME_SIZE:
            raise ValueError(f"frame len {len(raw)} != {DRIVE_FRAME_SIZE}")
        vals = struct.unpack(PACK_LE_V2, raw)
        return DriveFrame(
            vals[0],
            vals[1],
            vals[2],
            vals[3],
            vals[4],
            (vals[5], vals[6], vals[7]),
            (vals[8], vals[9], vals[10]),
            (vals[11], vals[12], vals[13], vals[14]),
            (vals[15], vals[16], vals[17], vals[18]),
            vals[19],
            vals[20],
            vals[21],
            (vals[22], vals[23], vals[24], vals[25]),
            vals[26],
        )


def parse_drive_frame(raw: bytes) -> Optional[DriveFrame]:
    if len(raw) != DRIVE_FRAME_SIZE or raw[0] != IMU_FRAME_MAGIC or raw[1] != DRIVE_FRAME_VER:
        return None
    fr = DriveFrame.unpack(raw)
    if crc16_ccitt_false(raw[:DRIVE_FRAME_CRC_LEN]) != fr.crc16:
        return None
    return fr
