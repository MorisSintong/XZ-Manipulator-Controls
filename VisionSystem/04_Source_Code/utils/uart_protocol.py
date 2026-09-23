"""
Modul Protokol UART Industrial — Binary Framed Packet untuk STM32 DMA
Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
Husein Alhamid (4212301035)

Spesifikasi Paket (18 byte):
    Byte 0-1  : Header           (0xAA 0x55)
    Byte 2    : Message Type     (0x01=Detection, 0x02=Heartbeat)
    Byte 3-4  : Object ID        (uint16, little-endian)
    Byte 5    : Class ID         (0=NonElco, 1=Benar, 2=Salah)
    Byte 6-7  : X Position mm    (int16, 0.1mm resolution, LE)
    Byte 8-9  : Y Position mm    (int16, 0.1mm resolution, LE)
    Byte 10-11: Angle deg*10     (int16, 0-3600, LE)
    Byte 12-13: Servo Correction (int16, deg*10, LE)
    Byte 14-15: CRC16-CCITT      (uint16, over bytes 0-13, LE)
    Byte 16-17: Tail             (0x0D 0x0A = \\r\\n)
"""

import struct
import numpy as np
from typing import Optional


# ─── Constants ───
HEADER = b'\xAA\x55'
TAIL = b'\x0D\x0A'
PACKET_SIZE = 18
PAYLOAD_CRC_REGION = 14  # bytes 0-13 are covered by CRC

MSG_TYPE_DETECTION = 0x01
MSG_TYPE_HEARTBEAT = 0x02

CLASS_NONELCO = 0
CLASS_BENAR = 1
CLASS_SALAH = 2


def compute_crc16_ccitt(data: bytes) -> int:
    """
    Compute CRC16-CCITT (polynomial 0x1021, init 0xFFFF).
    Matches the STM32 HAL CRC and ESP32 firmware implementation.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def encode_detection_packet(
    obj_id: int,
    class_id: int,
    x_mm: float,
    y_mm: float,
    angle_deg: float,
    correction_deg: float
) -> bytes:
    """
    Encode a detection/pick command into an 18-byte binary framed packet.

    Args:
        obj_id: Tracking object ID (0-65535).
        class_id: 0=NonElco, 1=Benar, 2=Salah.
        x_mm: Conveyor X position in mm (will be stored at 0.1mm resolution).
        y_mm: Conveyor Y position in mm.
        angle_deg: Orientation angle in degrees (0.0-360.0).
        correction_deg: Servo correction in degrees (0.0-180.0).

    Returns:
        18-byte binary packet.
    """
    # Convert to protocol units (0.1mm and 0.1deg resolution)
    x_units = int(round(x_mm * 10))
    y_units = int(round(y_mm * 10))
    angle_units = int(round(angle_deg * 10)) % 3600
    correction_units = int(round(correction_deg * 10))

    # Clamp values to int16 range
    x_units = max(-32768, min(32767, x_units))
    y_units = max(-32768, min(32767, y_units))
    correction_units = max(-32768, min(32767, correction_units))

    # Pack payload (bytes 0-13)
    payload = struct.pack('<2sBHBhhhh',
        HEADER,                     # bytes 0-1: header
        MSG_TYPE_DETECTION,         # byte 2: message type
        obj_id & 0xFFFF,            # bytes 3-4: object ID
        class_id & 0xFF,            # byte 5: class ID
        x_units,                    # bytes 6-7: X position
        y_units,                    # bytes 8-9: Y position
        angle_units,                # bytes 10-11: angle
        correction_units,           # bytes 12-13: correction
    )

    # Compute CRC16 over bytes 0-13
    crc = compute_crc16_ccitt(payload)

    # Append CRC16 + tail
    packet = payload + struct.pack('<H', crc) + TAIL

    return packet


def encode_heartbeat_packet() -> bytes:
    """
    Encode a heartbeat/status packet (18 bytes, zeroed data fields).
    """
    payload = struct.pack('<2sBHBhhhh',
        HEADER,
        MSG_TYPE_HEARTBEAT,
        0,      # obj_id
        0,      # class_id
        0, 0,   # x, y
        0, 0,   # angle, correction
    )
    crc = compute_crc16_ccitt(payload)
    return payload + struct.pack('<H', crc) + TAIL


def decode_detection_packet(data: bytes) -> dict:
    """
    Decode and validate an 18-byte binary framed packet.

    Args:
        data: Exactly 18 bytes.

    Returns:
        Dictionary with decoded fields.

    Raises:
        ValueError: If header/tail mismatch, wrong size, or CRC failure.
    """
    if len(data) != PACKET_SIZE:
        raise ValueError(f"Packet size mismatch: expected {PACKET_SIZE}, got {len(data)}")

    # Verify header
    if data[0:2] != HEADER:
        raise ValueError(f"Invalid header: {data[0:2].hex()}, expected {HEADER.hex()}")

    # Verify tail
    if data[16:18] != TAIL:
        raise ValueError(f"Invalid tail: {data[16:18].hex()}, expected {TAIL.hex()}")

    # Verify CRC16 over bytes 0-13
    payload = data[0:PAYLOAD_CRC_REGION]
    received_crc = struct.unpack('<H', data[14:16])[0]
    computed_crc = compute_crc16_ccitt(payload)
    if received_crc != computed_crc:
        raise ValueError(
            f"CRC16 mismatch: received 0x{received_crc:04X}, "
            f"computed 0x{computed_crc:04X}"
        )

    # Unpack fields
    msg_type = data[2]
    obj_id = struct.unpack('<H', data[3:5])[0]
    class_id = data[5]
    x_units, y_units, angle_units, correction_units = struct.unpack('<hhhh', data[6:14])

    return {
        'msg_type': msg_type,
        'obj_id': obj_id,
        'class_id': class_id,
        'x_mm': x_units / 10.0,
        'y_mm': y_units / 10.0,
        'angle_deg': angle_units / 10.0,
        'correction_deg': correction_units / 10.0,
    }


def format_debug_ascii(
    obj_id: int,
    class_id: int,
    x_mm: float,
    y_mm: float,
    angle_deg: float,
    correction_deg: float
) -> str:
    """
    Format detection data as human-readable ASCII for serial monitor debugging.

    Returns:
        String like: "DET:ID=42,CLS=1,X=125.3,Y=340.0,ANG=45.5,COR=314.5\\n"
    """
    class_labels = {0: "NONELCO", 1: "BENAR", 2: "SALAH"}
    cls_str = class_labels.get(class_id, str(class_id))
    return (
        f"DET:ID={obj_id},CLS={cls_str},"
        f"X={x_mm:.1f},Y={y_mm:.1f},"
        f"ANG={angle_deg:.1f},COR={correction_deg:.1f}\n"
    )


# ─── Pixel-to-Conveyor Coordinate Mapping ───

class PixelToConveyorMapper:
    """
    Maps pixel coordinates from the camera frame to physical conveyor
    coordinates in millimeters using a simple affine transformation
    (homography / scaling + offset).

    For a fixed overhead camera looking straight down at the conveyor:
        X_mm = (pixel_x - offset_x) * scale_x
        Y_mm = (pixel_y - offset_y) * scale_y

    For more complex setups (tilted camera), a full 3x3 homography matrix
    can be provided instead.

    Default calibration assumes:
        - Camera resolution: 640x480
        - Field of view covers ~320mm x 240mm of conveyor
        - Camera centered above conveyor
    """

    def __init__(
        self,
        scale_x: float = 0.5,       # mm per pixel (default: 320mm / 640px)
        scale_y: float = 0.5,       # mm per pixel (default: 240mm / 480px)
        offset_x: float = 0.0,      # pixel X offset (top-left corner)
        offset_y: float = 0.0,      # pixel Y offset (top-left corner)
        homography_matrix: Optional[np.ndarray] = None
    ):
        self.scale_x = scale_x
        self.scale_y = scale_y
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.homography = homography_matrix

    def pixel_to_mm(self, px: float, py: float) -> tuple:
        """
        Convert pixel coordinates to conveyor coordinates in mm.

        Args:
            px: Pixel X coordinate.
            py: Pixel Y coordinate.

        Returns:
            (x_mm, y_mm) tuple in millimeters.
        """
        if self.homography is not None:
            # Full projective homography transform
            pt = np.array([px, py, 1.0], dtype=np.float64)
            result = self.homography @ pt
            if abs(result[2]) > 1e-9:
                return (result[0] / result[2], result[1] / result[2])
            return (0.0, 0.0)

        # Simple affine scaling
        x_mm = (px - self.offset_x) * self.scale_x
        y_mm = (py - self.offset_y) * self.scale_y
        return (round(x_mm, 1), round(y_mm, 1))

    def set_calibration(
        self,
        scale_x: float,
        scale_y: float,
        offset_x: float = 0.0,
        offset_y: float = 0.0
    ):
        """Update the affine calibration parameters."""
        self.scale_x = scale_x
        self.scale_y = scale_y
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.homography = None

    def set_homography(self, matrix: np.ndarray):
        """Set a full 3x3 homography matrix for perspective correction."""
        assert matrix.shape == (3, 3), "Homography matrix must be 3x3"
        self.homography = matrix.astype(np.float64)
