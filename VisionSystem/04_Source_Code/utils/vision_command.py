"""Normalize physical vision outputs at the runtime command boundary."""

import math

from .uart_protocol import encode_detection_packet


def encode_runtime_detection_packet(
    obj_id, class_id, x_mm, y_mm, angle_deg, servo_correction_deg
):
    if class_id not in (0, 1, 2):
        raise ValueError(f"unsupported vision class ID {class_id}")
    if not all(math.isfinite(v) for v in (x_mm, y_mm, angle_deg, servo_correction_deg)):
        raise ValueError("non-finite vision coordinates/orientation/correction")
    angle_units = round(angle_deg * 10) % 3600
    correction_units = round(servo_correction_deg * 10) % 3600
    # Retain +180° for the exact half-turn tie; >180° uses its negative equivalent.
    if correction_units > 1800:
        correction_units -= 3600
    return encode_detection_packet(
        obj_id, class_id, x_mm, y_mm, angle_units / 10, correction_units / 10
    )
