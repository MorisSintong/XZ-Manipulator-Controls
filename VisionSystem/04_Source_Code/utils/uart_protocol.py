import struct

def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """Calculate CRC16-CCITT."""
    crc = init
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc = (crc << 1)
            crc &= 0xFFFF
    return crc

class PixelToConveyorMapper:
    """Handles camera-to-conveyor coordinate transformation."""
    
    def __init__(self, pixels_per_mm_x: float = 5.0, pixels_per_mm_y: float = 5.0, origin_offset_x_mm: float = 0.0, origin_offset_y_mm: float = 0.0):
        self.pixels_per_mm_x = pixels_per_mm_x
        self.pixels_per_mm_y = pixels_per_mm_y
        self.origin_offset_x_mm = origin_offset_x_mm
        self.origin_offset_y_mm = origin_offset_y_mm
        self.homography = None

    def set_homography(self, homography_matrix):
        """Set an optional homography matrix for advanced calibration."""
        self.homography = homography_matrix

    def pixel_to_mm(self, px_x: float, px_y: float) -> tuple[float, float]:
        """Convert pixel coordinates to conveyor mm coordinates."""
        if self.homography is not None:
            # Assumes homography is a 3x3 numpy array
            import numpy as np
            pt = np.array([px_x, px_y, 1.0])
            mapped_pt = np.dot(self.homography, pt)
            mapped_pt = mapped_pt / mapped_pt[2]
            return float(mapped_pt[0]), float(mapped_pt[1])
            
        x_mm = (px_x / self.pixels_per_mm_x) + self.origin_offset_x_mm
        y_mm = (px_y / self.pixels_per_mm_y) + self.origin_offset_y_mm
        return x_mm, y_mm


def encode_detection_packet(obj_id: int, class_id: int, x_mm: float, y_mm: float, angle_deg: float, servo_correction_deg: float) -> bytes:
    """Encode a detection/pick command packet with CRC16."""
    header = b'\xAA\x55'
    msg_type = 0x01
    
    # Scale and cast
    x_val = int(round(x_mm * 10))
    y_val = int(round(y_mm * 10))
    angle_val = int(round(angle_deg * 10))
    servo_val = int(round(servo_correction_deg * 10))
    
    # <B H B h h h h
    # B = msg_type (1)
    # H = obj_id (2)
    # B = class_id (1)
    # h = x (2)
    # h = y (2)
    # h = angle (2)
    # h = servo (2)
    # Total payload: 12 bytes. + 2 header bytes = 14 bytes to CRC
    payload = struct.pack('<B H B h h h h', msg_type, obj_id, class_id, x_val, y_val, angle_val, servo_val)
    
    data_to_crc = header + payload
    crc = crc16_ccitt(data_to_crc)
    
    crc_bytes = struct.pack('<H', crc)
    tail = b'\x0D\x0A'
    
    return data_to_crc + crc_bytes + tail


def decode_detection_packet(data: bytes) -> dict | None:
    """Decode and verify CRC of a detection packet."""
    if len(data) != 18:
        return None
        
    if data[0:2] != b'\xAA\x55':
        return None
        
    if data[16:18] != b'\x0D\x0A':
        return None
        
    data_to_crc = data[:14]
    expected_crc = struct.unpack('<H', data[14:16])[0]
    
    actual_crc = crc16_ccitt(data_to_crc)
    if expected_crc != actual_crc:
        return None
        
    msg_type, obj_id, class_id, x_val, y_val, angle_val, servo_val = struct.unpack('<B H B h h h h', data[2:14])
    
    return {
        'msg_type': msg_type,
        'obj_id': obj_id,
        'class_id': class_id,
        'x_mm': x_val / 10.0,
        'y_mm': y_val / 10.0,
        'angle_deg': angle_val / 10.0,
        'servo_correction_deg': servo_val / 10.0
    }


def encode_heartbeat_packet(seq: int = 0) -> bytes:
    """Encode a heartbeat/status packet with CRC16."""
    header = b'\xAA\x55'
    msg_type = 0x02
    
    # Dummy zero values for fields that don't matter in heartbeat
    class_id = 0
    x_val = 0
    y_val = 0
    angle_val = 0
    servo_val = 0
    
    payload = struct.pack('<B H B h h h h', msg_type, seq, class_id, x_val, y_val, angle_val, servo_val)
    
    data_to_crc = header + payload
    crc = crc16_ccitt(data_to_crc)
    
    crc_bytes = struct.pack('<H', crc)
    tail = b'\x0D\x0A'
    
    return data_to_crc + crc_bytes + tail


def format_ascii_debug(obj_id: int, class_id: int, x_mm: float, y_mm: float, angle_deg: float, servo_deg: float) -> str:
    """Human-readable ASCII fallback for debug monitoring."""
    return f"OBJ:{obj_id} CLS:{class_id} X:{x_mm:.1f} Y:{y_mm:.1f} ANG:{angle_deg:.1f} SRV:{servo_deg:.1f}\r\n"
