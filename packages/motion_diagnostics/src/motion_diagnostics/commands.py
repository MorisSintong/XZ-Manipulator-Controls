"""Deployed 18-byte vision command contract (CRC includes AA55)."""

import struct
from dataclasses import astuple, dataclass

from .crc import crc16_ccitt

PAYLOAD = struct.Struct("<BHBhhhh")


@dataclass(frozen=True)
class Command:
    command_type: int = 1
    obj_id: int = 0
    class_id: int = 1
    x_01mm: int = 0
    y_01mm: int = 0
    angle_01deg: int = 0
    corr_01deg: int = 0

    def encode(self) -> bytes:
        if self.command_type not in (1, 2, 3, 4):
            raise ValueError("unsupported command")
        if (
            not 0 <= self.obj_id <= 65535
            or not -32768 <= self.x_01mm <= 32767
            or not -32768 <= self.y_01mm <= 32767
        ):
            raise ValueError("identity/coordinates outside wire bounds")
        if (
            self.class_id not in (0, 1, 2)
            or not 0 <= self.angle_01deg < 3600
            or not -1800 <= self.corr_01deg <= 1800
        ):
            raise ValueError("class/angle/correction out of range")
        body = b"\xaa\x55" + PAYLOAD.pack(*astuple(self))
        return body + struct.pack("<H", crc16_ccitt(body)) + b"\r\n"


def decode_command(frame: bytes) -> Command:
    if len(frame) != 18 or frame[:2] != b"\xaa\x55" or frame[-2:] != b"\r\n":
        raise ValueError("command framing")
    if crc16_ccitt(frame[:14]) != struct.unpack_from("<H", frame, 14)[0]:
        raise ValueError("command CRC")
    command = Command(*PAYLOAD.unpack_from(frame, 2))
    command.encode()
    return command


def encode_detection_packet(
    obj_id, class_id, x_mm, y_mm, angle_deg, servo_correction_deg
):
    return Command(
        1,
        obj_id,
        class_id,
        round(x_mm * 10),
        round(y_mm * 10),
        round(angle_deg * 10) % 3600,
        round(servo_correction_deg * 10),
    ).encode()


def control_packet(command_type: int, obj_id: int = 0) -> bytes:
    if command_type not in (2, 3, 4):
        raise ValueError("control type must be heartbeat, abort or rehome")
    return Command(command_type=command_type, obj_id=obj_id, class_id=0).encode()
