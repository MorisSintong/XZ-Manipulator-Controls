"""
===============================================================================
  verify_vision_system.py — Automated Test Suite for VisionSystem Remediation
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)

  Tests:
    1. Continuous angle calculation accuracy (±2.5° tolerance)
    2. UART protocol serialization, CRC16 checksum, corruption rejection
    3. Conveyor tracker debounce (1 pick per object across 60 frames)
    4. Launcher portability (no hardcoded user paths)
===============================================================================
"""

import os
import sys
import math
import struct
import unittest
import glob
from pathlib import Path

# Add source code to path
ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.insert(0, str(ROOT_DIR / "04_Source_Code"))

import numpy as np
import cv2

from utils.angle_calculator import (
    hitung_sudut_kontur,
    normalisasi_sudut_360,
    hitung_koreksi_terpendek,
    evaluasi_qc_dan_servo,
)
from utils.uart_protocol import (
    encode_detection_packet,
    decode_detection_packet,
    compute_crc16_ccitt,
    encode_heartbeat_packet,
    format_debug_ascii,
    PixelToConveyorMapper,
    PACKET_SIZE,
    HEADER,
    TAIL,
    MSG_TYPE_DETECTION,
    MSG_TYPE_HEARTBEAT,
)
from utils.conveyor_tracker import ConveyorTracker, Detection


# ═══════════════════════════════════════════════════════════════════════
# Helper: Generate synthetic capacitor ROI images at a known angle
# ═══════════════════════════════════════════════════════════════════════

def create_synthetic_capacitor_roi(angle_deg: float, size: int = 120) -> np.ndarray:
    """
    Create a synthetic capacitor ROI image at a known orientation.

    The capacitor is modeled as an elongated dark rectangle (body) on a
    light background, with a bright rectangular region at one end
    (lead pins / metallic legs).

    Convention (matching the system):
        0°   = Legs at bottom
        90°  = Legs at right
        180° = Legs at top
        270° = Legs at left

    Args:
        angle_deg: Desired orientation in degrees [0, 360).
        size: Size of the square ROI image.

    Returns:
        BGR image (size x size x 3).
    """
    # Light gray background (simulates conveyor belt)
    temp = np.full((size, size), 200, dtype=np.uint8)
    center = (size // 2, size // 2)

    # Capacitor body dimensions (elongated)
    body_w = size // 5       # narrow (width of cylinder)
    body_h = size * 2 // 3   # tall (length of cylinder)
    leg_h = body_h // 4      # length of the bright leg region

    bx1 = center[0] - body_w // 2
    bx2 = center[0] + body_w // 2
    by1 = center[1] - body_h // 2
    by2 = center[1] + body_h // 2

    # Dark capacitor body
    cv2.rectangle(temp, (bx1, by1), (bx2, by2), 50, -1)

    # Bright metallic legs at the bottom end (canonical 0° = legs at bottom)
    cv2.rectangle(temp, (bx1 - 2, by2 - leg_h), (bx2 + 2, by2 + 4), 240, -1)

    # Rotate the capacitor to desired angle.
    # OpenCV getRotationMatrix2D: positive angle in image coords (Y-down)
    # moves a bottom point to the RIGHT, which matches our screen convention
    # (0°=down, 90°=right, CW positive). So we use +angle_deg directly.
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    rotated = cv2.warpAffine(temp, M, (size, size), borderValue=200)

    # Convert to BGR
    bgr = np.stack([rotated, rotated, rotated], axis=-1)
    return bgr


def angular_error(measured: float, expected: float) -> float:
    """Compute the shortest angular distance between two angles."""
    diff = (measured - expected + 180.0) % 360.0 - 180.0
    return abs(diff)


# ═══════════════════════════════════════════════════════════════════════
# TEST GROUP 1: Continuous Angle Calculation
# ═══════════════════════════════════════════════════════════════════════

class TestContinuousAngle(unittest.TestCase):
    """
    Verify that hitung_sudut_kontur() produces continuous angles
    across [0°, 360°) — NOT quantized to 4 discrete buckets.
    """

    TOLERANCE_DEG = 2.5  # Maximum allowed error per REMEDIATION_PLAN_AI.md

    def test_normalisasi_sudut_360(self):
        """Verify angle normalization to [0, 360)."""
        self.assertAlmostEqual(normalisasi_sudut_360(0.0), 0.0, places=1)
        self.assertAlmostEqual(normalisasi_sudut_360(360.0), 0.0, places=1)
        self.assertAlmostEqual(normalisasi_sudut_360(-90.0), 270.0, places=1)
        self.assertAlmostEqual(normalisasi_sudut_360(-270.0), 90.0, places=1)
        self.assertAlmostEqual(normalisasi_sudut_360(450.0), 90.0, places=1)

    def test_koreksi_terpendek(self):
        """Verify CW correction calculation."""
        # 90° actual → correction should be 270° CW to reach 0°
        _, dev, kor, _ = hitung_koreksi_terpendek(90.0, 0.0)
        self.assertAlmostEqual(kor, 270.0, places=1)

        # 0° actual → no correction needed
        _, dev, kor, arah = hitung_koreksi_terpendek(0.0, 0.0)
        self.assertAlmostEqual(kor, 0.0, places=1)
        self.assertEqual(arah, "DIAM")

    def test_angle_not_quantized(self):
        """
        CRITICAL TEST: Verify the angle calculator does NOT produce
        only 0°, 90°, 180°, 270° for non-orthogonal inputs.

        This is the primary regression test for the quantization bug.
        """
        non_orthogonal_angles = [15.0, 30.0, 45.0, 60.0, 135.0, 225.0, 315.0]
        discrete_buckets = {0.0, 90.0, 180.0, 270.0}

        results = []
        for target in non_orthogonal_angles:
            roi = create_synthetic_capacitor_roi(target, size=120)
            measured = hitung_sudut_kontur(roi)
            results.append((target, measured))

        # At least 4 of 7 non-orthogonal angles must NOT fall into discrete buckets
        non_quantized_count = sum(
            1 for _, measured in results
            if measured not in discrete_buckets
        )

        self.assertGreaterEqual(
            non_quantized_count, 4,
            f"Too many angles quantized to discrete buckets! "
            f"Results: {results}"
        )

    def test_continuous_angle_accuracy(self):
        """
        Test continuous angle accuracy at multiple orientations.
        Each measured angle must be within ±2.5° of ground truth.
        """
        test_angles = [0.0, 15.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]

        for target in test_angles:
            with self.subTest(target_angle=target):
                roi = create_synthetic_capacitor_roi(target, size=120)
                measured = hitung_sudut_kontur(roi)

                error = angular_error(measured, target)
                self.assertLessEqual(
                    error, self.TOLERANCE_DEG,
                    f"Angle error {error:.2f}° exceeds tolerance {self.TOLERANCE_DEG}° "
                    f"for target={target}°, measured={measured}°"
                )

    def test_evaluasi_no_class_name_fallback(self):
        """
        Verify that evaluasi_qc_dan_servo does NOT use class name
        as a shortcut for physical angle calculation.

        Both 'Elco_Benar' and 'Elco_Salah' at 45° should produce
        the same physical angle measurement.
        """
        roi_45 = create_synthetic_capacitor_roi(45.0, size=120)

        result_benar = evaluasi_qc_dan_servo(1, "Elco_Benar", roi_45)
        result_salah = evaluasi_qc_dan_servo(2, "Elco_Salah", roi_45)

        sudut_benar = result_benar[0]  # sudut_act
        sudut_salah = result_salah[0]  # sudut_act

        # Both should measure roughly the same physical angle
        angle_diff = angular_error(sudut_benar, sudut_salah)
        self.assertLessEqual(
            angle_diff, 5.0,
            f"Class name fallback detected! "
            f"Benar={sudut_benar}°, Salah={sudut_salah}° "
            f"(diff={angle_diff}°) for same 45° ROI"
        )

    def test_empty_roi_returns_float(self):
        """Verify edge case: empty/tiny ROI returns a valid float."""
        tiny = np.zeros((2, 2, 3), dtype=np.uint8)
        result = hitung_sudut_kontur(tiny)
        self.assertIsInstance(result, float)

    def test_evaluasi_returns_structured_data(self):
        """Verify evaluasi_qc_dan_servo returns dict for UART encoding."""
        roi = create_synthetic_capacitor_roi(0.0, size=80)
        result = evaluasi_qc_dan_servo(1, "Elco_Benar", roi)
        uart_data = result[7]  # 8th return value
        self.assertIsInstance(uart_data, dict)
        self.assertIn('class_id', uart_data)
        self.assertIn('angle_deg', uart_data)
        self.assertIn('correction_deg', uart_data)


# ═══════════════════════════════════════════════════════════════════════
# TEST GROUP 2: UART Protocol Serialization & CRC16
# ═══════════════════════════════════════════════════════════════════════

class TestUARTProtocol(unittest.TestCase):
    """
    Verify binary framed packet encoding, CRC16 checksums,
    and corruption detection.
    """

    def test_packet_size(self):
        """Verify packet is exactly 18 bytes."""
        packet = encode_detection_packet(
            obj_id=1, class_id=1,
            x_mm=125.3, y_mm=340.0,
            angle_deg=45.5, correction_deg=314.5
        )
        self.assertEqual(len(packet), PACKET_SIZE)

    def test_packet_header_tail(self):
        """Verify header and tail bytes."""
        packet = encode_detection_packet(
            obj_id=1, class_id=2,
            x_mm=100.0, y_mm=200.0,
            angle_deg=180.0, correction_deg=180.0
        )
        self.assertEqual(packet[0:2], HEADER)
        self.assertEqual(packet[16:18], TAIL)

    def test_encode_decode_roundtrip(self):
        """Verify encode → decode produces identical values."""
        test_cases = [
            (42, 1, 125.3, 340.0, 45.5, 314.5),
            (0, 0, 0.0, 0.0, 0.0, 0.0),
            (65535, 2, -100.0, 500.0, 359.9, 180.0),
            (1, 1, 320.0, 240.0, 0.0, 0.0),
        ]
        for obj_id, cls_id, x, y, ang, cor in test_cases:
            with self.subTest(obj_id=obj_id, cls=cls_id):
                packet = encode_detection_packet(obj_id, cls_id, x, y, ang, cor)
                decoded = decode_detection_packet(packet)

                self.assertEqual(decoded['obj_id'], obj_id)
                self.assertEqual(decoded['class_id'], cls_id)
                self.assertAlmostEqual(decoded['x_mm'], x, places=0)
                self.assertAlmostEqual(decoded['y_mm'], y, places=0)
                self.assertAlmostEqual(decoded['angle_deg'], ang, delta=0.1)
                self.assertAlmostEqual(decoded['correction_deg'], cor, delta=0.1)

    def test_crc16_consistency(self):
        """Verify CRC16-CCITT produces consistent results."""
        data = b'\xAA\x55\x01\x00\x01\x01\xE8\x03\xD0\x07\xC2\x01\x00\x00'
        crc1 = compute_crc16_ccitt(data)
        crc2 = compute_crc16_ccitt(data)
        self.assertEqual(crc1, crc2)
        self.assertIsInstance(crc1, int)
        self.assertGreater(crc1, 0)

    def test_corruption_detection(self):
        """Verify that single-byte corruption is detected by CRC16."""
        packet = encode_detection_packet(
            obj_id=42, class_id=1,
            x_mm=100.0, y_mm=200.0,
            angle_deg=45.0, correction_deg=315.0
        )

        # Corrupt a data byte (byte 6 = X position LSB)
        corrupted = bytearray(packet)
        corrupted[6] ^= 0xFF  # Flip all bits
        corrupted = bytes(corrupted)

        with self.assertRaises(ValueError, msg="CRC should detect corruption"):
            decode_detection_packet(corrupted)

    def test_invalid_header_rejected(self):
        """Verify packets with wrong header are rejected."""
        packet = encode_detection_packet(1, 1, 0, 0, 0, 0)
        bad_header = b'\x00\x00' + packet[2:]
        with self.assertRaises(ValueError):
            decode_detection_packet(bad_header)

    def test_invalid_size_rejected(self):
        """Verify packets with wrong size are rejected."""
        with self.assertRaises(ValueError):
            decode_detection_packet(b'\xAA\x55\x01')

    def test_heartbeat_packet(self):
        """Verify heartbeat packet encoding and decoding."""
        packet = encode_heartbeat_packet()
        self.assertEqual(len(packet), PACKET_SIZE)
        decoded = decode_detection_packet(packet)
        self.assertEqual(decoded['msg_type'], MSG_TYPE_HEARTBEAT)

    def test_debug_ascii_format(self):
        """Verify ASCII debug format produces readable string."""
        msg = format_debug_ascii(42, 1, 125.3, 340.0, 45.5, 314.5)
        self.assertIn("DET:", msg)
        self.assertIn("ID=42", msg)
        self.assertIn("BENAR", msg)
        self.assertIn("125.3", msg)
        self.assertTrue(msg.endswith("\n"))

    def test_pixel_to_mm_mapping(self):
        """Verify pixel-to-mm coordinate conversion."""
        mapper = PixelToConveyorMapper(scale_x=0.5, scale_y=0.5)
        x_mm, y_mm = mapper.pixel_to_mm(320, 240)
        self.assertAlmostEqual(x_mm, 160.0, places=1)
        self.assertAlmostEqual(y_mm, 120.0, places=1)

    def test_pixel_to_mm_with_offset(self):
        """Verify pixel-to-mm with calibration offset."""
        mapper = PixelToConveyorMapper(scale_x=0.5, scale_y=0.5,
                                        offset_x=100.0, offset_y=50.0)
        x_mm, y_mm = mapper.pixel_to_mm(100, 50)
        self.assertAlmostEqual(x_mm, 0.0, places=1)
        self.assertAlmostEqual(y_mm, 0.0, places=1)


# ═══════════════════════════════════════════════════════════════════════
# TEST GROUP 3: Conveyor Tracker Debounce
# ═══════════════════════════════════════════════════════════════════════

class TestConveyorTrackerDebounce(unittest.TestCase):
    """
    Verify that the ConveyorTracker generates exactly 1 pick command
    per object, even when the object is visible across many frames.
    """

    def test_single_object_60_frames_one_pick(self):
        """
        CRITICAL TEST: Simulate 60 frames of a single capacitor
        moving along Y-axis. Assert exactly 1 UART pick command.
        """
        tracker = ConveyorTracker(
            max_disappeared=10,
            distance_threshold=80.0,
            inspection_line_y=200,
            min_frames_before_eval=3,
        )

        total_picks = 0

        for frame_idx in range(60):
            # Capacitor moves downward (increasing Y) across frames
            y_pos = 50 + frame_idx * 5  # starts at y=50, moves down
            bbox = (100, y_pos, 150, y_pos + 60)

            det = Detection(
                class_id=1,
                class_name="Elco_Benar",
                confidence=0.95,
                bbox=bbox,
                angle_deg=0.0,
                correction_deg=0.0,
                status_qc="BENAR",
                aksi="PASS",
            )

            tracker.update([det])
            picks = tracker.get_pending_picks()
            total_picks += len(picks)

        self.assertEqual(
            total_picks, 1,
            f"Expected exactly 1 pick command across 60 frames, "
            f"got {total_picks}"
        )

    def test_two_objects_two_picks(self):
        """Two separate capacitors should generate exactly 2 pick commands."""
        tracker = ConveyorTracker(
            max_disappeared=10,
            distance_threshold=60.0,
            inspection_line_y=150,
            min_frames_before_eval=2,
        )

        total_picks = 0

        for frame_idx in range(40):
            y_pos = 50 + frame_idx * 5
            detections = []

            # Object A (left side)
            detections.append(Detection(
                class_id=1, class_name="Elco_Benar", confidence=0.90,
                bbox=(50, y_pos, 100, y_pos + 50),
                angle_deg=0.0, correction_deg=0.0,
                status_qc="BENAR", aksi="PASS",
            ))

            # Object B (right side, staggered start)
            if frame_idx >= 5:
                y_b = 50 + (frame_idx - 5) * 5
                detections.append(Detection(
                    class_id=2, class_name="Elco_Salah", confidence=0.88,
                    bbox=(250, y_b, 300, y_b + 50),
                    angle_deg=180.0, correction_deg=180.0,
                    status_qc="SALAH", aksi="REORIENTASI",
                ))

            tracker.update(detections)
            picks = tracker.get_pending_picks()
            total_picks += len(picks)

        self.assertEqual(
            total_picks, 2,
            f"Expected exactly 2 pick commands for 2 objects, got {total_picks}"
        )

    def test_no_pick_before_inspection_line(self):
        """Object that never reaches inspection line should not be picked."""
        tracker = ConveyorTracker(
            inspection_line_y=500,  # Very far down
            min_frames_before_eval=2,
        )

        total_picks = 0
        for frame_idx in range(20):
            # Object stays in upper part of frame (y < 100)
            bbox = (100, 10 + frame_idx * 2, 150, 60 + frame_idx * 2)
            det = Detection(
                class_id=1, class_name="Elco_Benar", confidence=0.95,
                bbox=bbox, angle_deg=0.0, correction_deg=0.0,
                status_qc="BENAR", aksi="PASS",
            )
            tracker.update([det])
            total_picks += len(tracker.get_pending_picks())

        self.assertEqual(total_picks, 0, "No pick should occur above inspection line")

    def test_disappeared_object_removed(self):
        """Object that disappears should be cleaned up after max_disappeared frames."""
        tracker = ConveyorTracker(
            max_disappeared=5,
            inspection_line_y=200,
        )

        # Object appears for 3 frames
        for i in range(3):
            det = Detection(
                class_id=1, class_name="Elco_Benar", confidence=0.9,
                bbox=(100, 100, 150, 150),
                angle_deg=0.0, correction_deg=0.0,
                status_qc="BENAR", aksi="PASS",
            )
            tracker.update([det])

        # Object disappears for 10 frames
        for i in range(10):
            tracker.update([])

        # All tracks should be gone
        self.assertEqual(len(tracker.active_tracks), 0)

    def test_tracker_reset(self):
        """Verify tracker reset clears all state."""
        tracker = ConveyorTracker()
        det = Detection(
            class_id=1, class_name="Elco_Benar", confidence=0.9,
            bbox=(100, 250, 150, 300),
            angle_deg=0.0, correction_deg=0.0,
            status_qc="BENAR", aksi="PASS",
        )
        tracker.update([det])
        tracker.update([det])
        tracker.update([det])
        tracker.reset()

        self.assertEqual(len(tracker.active_tracks), 0)
        self.assertEqual(len(tracker.get_pending_picks()), 0)


# ═══════════════════════════════════════════════════════════════════════
# TEST GROUP 4: Launcher Portability
# ═══════════════════════════════════════════════════════════════════════

class TestLauncherPortability(unittest.TestCase):
    """
    Verify that no .bat launcher files contain hardcoded user paths.
    """

    FORBIDDEN_PATTERNS = [
        r"C:\Users\HUSEN",
        r"C:/Users/HUSEN",
        "C:\\Users\\HUSEN",
    ]

    def _get_bat_files(self):
        """Collect all .bat files in the VisionSystem directory."""
        bat_files = []
        vision_dir = ROOT_DIR
        for pattern in ["*.bat", "**/*.bat"]:
            bat_files.extend(vision_dir.glob(pattern))
        return list(set(bat_files))

    def test_no_hardcoded_paths_in_bat_files(self):
        """
        CRITICAL TEST: All .bat files must NOT contain hardcoded
        user directory paths like C:\\Users\\HUSEN.
        """
        bat_files = self._get_bat_files()
        self.assertGreater(len(bat_files), 0, "No .bat files found to test!")

        violations = []
        for bat_file in bat_files:
            try:
                content = bat_file.read_text(encoding='utf-8', errors='ignore')
            except Exception:
                content = bat_file.read_text(encoding='latin-1', errors='ignore')

            for pattern in self.FORBIDDEN_PATTERNS:
                if pattern.lower() in content.lower():
                    violations.append((bat_file.name, pattern))

        self.assertEqual(
            len(violations), 0,
            f"Hardcoded paths found in launchers: {violations}"
        )

    def test_bat_files_use_python_cmd(self):
        """Verify that .bat files use %PYTHON_CMD% or 'python' command."""
        bat_files = self._get_bat_files()
        for bat_file in bat_files:
            content = bat_file.read_text(encoding='utf-8', errors='ignore')
            # Each bat file should use either %PYTHON_CMD% or bare 'python'
            uses_python_cmd = "PYTHON_CMD" in content or "python " in content.lower()
            self.assertTrue(
                uses_python_cmd,
                f"{bat_file.name} does not appear to use PYTHON_CMD or python"
            )


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 70)
    print("  VisionSystem Remediation — Automated Verification Suite")
    print("  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas")
    print("=" * 70)
    print()

    unittest.main(verbosity=2)
