"""
===============================================================================
  verify_vision_system.py — AUTOMATED VERIFICATION TEST SUITE
  VisionSystem Remediation Plan — Full Compliance Check
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
===============================================================================

  Tests performed:
  1. Continuous angle calculation (non-orthogonal angles, ≤ ±2.5° error)
  2. UART protocol serialization, CRC16, and corruption rejection
  3. Conveyor tracker debounce (single object → exactly 1 UART trigger)
  4. Launcher batch files contain no hardcoded user directories
  5. No class-name fallback in angle calculation logic
  6. Firmware: no NVS flash writes, no blocking delay() in real-time loop

  Run: python tests/verify_vision_system.py
  All tests must pass with exit code 0.
===============================================================================
"""

import os
import sys
import io
import math
import struct
import unittest
import re
from pathlib import Path

# Force UTF-8 output on Windows
if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
if sys.stderr.encoding != 'utf-8':
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# Setup path
ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.insert(0, str(ROOT_DIR / "04_Source_Code"))

import numpy as np
import cv2

from utils.angle_calculator import (
    hitung_sudut_kontur,
    evaluasi_qc_dan_servo,
    normalisasi_sudut_360,
    hitung_koreksi_terpendek,
)
from utils.uart_protocol import (
    crc16_ccitt,
    encode_detection_packet,
    decode_detection_packet,
    encode_heartbeat_packet,
    format_ascii_debug,
    PixelToConveyorMapper,
)
from utils.conveyor_tracker import ConveyorTracker


# =============================================================================
# Helper: Generate synthetic capacitor ROI image at a known angle
# =============================================================================
def make_capacitor_roi(angle_deg: float, size: int = 120) -> np.ndarray:
    """
    Generate a synthetic capacitor-like ROI image rotated to `angle_deg`.

    Convention: 0° = pin/lead pointing DOWN (bottom of image).
    
    The image has:
    - BRIGHT background (simulating white/light conveyor belt, ~200 gray)
    - DARK elongated capacitor body (~50 gray)
    - BRIGHT metallic pins (~220 gray) on the lead end
    - DARKER markings (~30 gray) on the body base end

    The angle uses the system convention:
        0°   = lead at bottom  →  pin end points downward
        90°  = lead at right   →  pin end points right
        180° = lead at top     →  pin end points up
        270° = lead at left    →  pin end points left
    """
    # Bright background (simulates conveyor belt surface)
    img = np.full((size, size, 3), 200, dtype=np.uint8)

    cx, cy = size // 2, size // 2

    # Capsule body dimensions
    body_length = int(size * 0.65)
    body_width = int(size * 0.22)

    # We draw the capacitor HORIZONTALLY first (pin on RIGHT side),
    # then rotate. The rotation maps system angle to screen orientation.
    # 
    # System angle convention:
    #   0°   = pin DOWN   → body vertical, pin at bottom
    #   90°  = pin RIGHT  → body horizontal, pin at right  
    #   180° = pin UP     → body vertical, pin at top
    #   270° = pin LEFT   → body horizontal, pin at left
    #
    # OpenCV rotation: positive angle = counter-clockwise
    # At 0° (pin down), the body was drawn horizontal with pin right,
    # so we need to rotate -90° (or +270°) to get pin pointing down.
    # General: rotate_deg = angle_deg - 90
    # cv2.getRotationMatrix2D uses positive = CCW in screen coordinates.
    # Pin starts on RIGHT. For angle=0 (pin down), need CW 90 = CCW -90.
    
    rotate_deg = angle_deg - 90.0

    # Canvas for the unrotated body (horizontal orientation)
    body_img = np.full((size, size, 3), 200, dtype=np.uint8)

    half_l = body_length // 2
    half_w = body_width // 2

    # Capacitor body: dark gray rectangle (horizontal)
    cv2.rectangle(body_img,
                  (cx - half_l, cy - half_w),
                  (cx + half_l, cy + half_w),
                  (50, 50, 50), -1)

    # Pin/lead end: bright metallic patch on the RIGHT side
    pin_len = int(body_length * 0.22)
    cv2.rectangle(body_img,
                  (cx + half_l - pin_len, cy - half_w + 2),
                  (cx + half_l, cy + half_w - 2),
                  (220, 220, 220), -1)

    # Body base/markings: extra dark on the LEFT side (non-pin end)
    cv2.rectangle(body_img,
                  (cx - half_l, cy - half_w),
                  (cx - half_l + pin_len, cy + half_w),
                  (30, 30, 30), -1)

    # Rotate the image
    M = cv2.getRotationMatrix2D((cx, cy), rotate_deg, 1.0)
    rotated = cv2.warpAffine(body_img, M, (size, size),
                              borderValue=(200, 200, 200))

    return rotated




# =============================================================================
# TEST 1: Continuous Angle Calculation
# =============================================================================
class TestContinuousAngle(unittest.TestCase):
    """
    Verify that hitung_sudut_kontur() produces continuous angles
    across [0°, 360°) with ≤ ±2.5° error on non-orthogonal rotations.
    """

    def _angle_error(self, measured: float, expected: float) -> float:
        """Compute shortest angular distance between two angles."""
        diff = (measured - expected + 180.0) % 360.0 - 180.0
        return abs(diff)

    def test_continuous_angles_non_orthogonal(self):
        """Test angles at 15°, 30°, 45°, 60°, 135° as required by remediation plan."""
        test_angles = [15.0, 30.0, 45.0, 60.0, 135.0]
        max_error_threshold = 2.5
        errors = []

        for expected_angle in test_angles:
            roi = make_capacitor_roi(expected_angle, size=150)
            measured = hitung_sudut_kontur(roi)
            error = self._angle_error(measured, expected_angle)
            errors.append((expected_angle, measured, error))

            self.assertLessEqual(
                error, max_error_threshold,
                f"Angle {expected_angle}°: measured {measured}°, error {error:.2f}° > {max_error_threshold}°"
            )

        print("\n  ┌─ Continuous Angle Test Results ─┐")
        for exp, meas, err in errors:
            status = "✓" if err <= max_error_threshold else "✗"
            print(f"  │ [{status}] Target: {exp:6.1f}° → Measured: {meas:6.1f}° | Error: {err:.2f}° │")
        print("  └──────────────────────────────────┘")

    def test_orthogonal_angles(self):
        """Also verify standard 0°, 90°, 180°, 270° still work."""
        test_angles = [0.0, 90.0, 180.0, 270.0]

        for expected_angle in test_angles:
            roi = make_capacitor_roi(expected_angle, size=150)
            measured = hitung_sudut_kontur(roi)
            error = self._angle_error(measured, expected_angle)

            self.assertLessEqual(
                error, 5.0,
                f"Orthogonal {expected_angle}°: measured {measured}°, error {error:.2f}°"
            )

    def test_angle_output_range(self):
        """Verify all outputs are in [0, 360)."""
        for angle in [0, 30, 45, 90, 120, 180, 225, 270, 315, 350]:
            roi = make_capacitor_roi(angle, size=120)
            result = hitung_sudut_kontur(roi)
            self.assertGreaterEqual(result, 0.0, f"Angle {result} < 0")
            self.assertLess(result, 360.0, f"Angle {result} >= 360")

    def test_normalisasi_sudut(self):
        """Test angle normalization edge cases."""
        self.assertAlmostEqual(normalisasi_sudut_360(-270), 90.0)
        self.assertAlmostEqual(normalisasi_sudut_360(-90), 270.0)
        self.assertAlmostEqual(normalisasi_sudut_360(450), 90.0)
        self.assertAlmostEqual(normalisasi_sudut_360(0), 0.0)
        self.assertAlmostEqual(normalisasi_sudut_360(360), 0.0)

    def test_no_class_name_fallback_for_angle(self):
        """
        Verify that evaluasi_qc_dan_servo does NOT use class_name
        as a shortcut for angle calculation. Two different class names
        with the same ROI must produce the same angle.
        """
        roi = make_capacitor_roi(45.0, size=120)

        result_benar = evaluasi_qc_dan_servo(1, "Elco_Benar", roi)
        result_salah = evaluasi_qc_dan_servo(2, "Elco_Salah", roi)

        # Both should compute the same angle from the same ROI
        angle_benar = result_benar[0]  # sudut_act
        angle_salah = result_salah[0]

        self.assertAlmostEqual(
            angle_benar, angle_salah, places=1,
            msg=f"Class name is affecting angle! Benar={angle_benar}, Salah={angle_salah}"
        )


# =============================================================================
# TEST 2: UART Protocol Serialization & CRC16
# =============================================================================
class TestUARTProtocol(unittest.TestCase):
    """Verify binary packet encoding, CRC16, and corruption detection."""

    def test_encode_decode_roundtrip(self):
        """Encode a packet, decode it, verify all fields match."""
        pkt = encode_detection_packet(
            obj_id=42, class_id=2,
            x_mm=125.3, y_mm=67.8,
            angle_deg=135.5, servo_correction_deg=44.5
        )

        self.assertEqual(len(pkt), 18, "Packet must be exactly 18 bytes")
        self.assertEqual(pkt[0:2], b'\xAA\x55', "Header mismatch")
        self.assertEqual(pkt[16:18], b'\x0D\x0A', "Tail mismatch")

        decoded = decode_detection_packet(pkt)
        self.assertIsNotNone(decoded, "Valid packet decoded as None")

        self.assertEqual(decoded['msg_type'], 0x01)
        self.assertEqual(decoded['obj_id'], 42)
        self.assertEqual(decoded['class_id'], 2)
        self.assertAlmostEqual(decoded['x_mm'], 125.3, places=1)
        self.assertAlmostEqual(decoded['y_mm'], 67.8, places=1)
        self.assertAlmostEqual(decoded['angle_deg'], 135.5, places=1)
        self.assertAlmostEqual(decoded['servo_correction_deg'], 44.5, places=1)

    def test_crc16_corruption_rejection(self):
        """Inject byte corruption and verify packet is rejected."""
        pkt = encode_detection_packet(
            obj_id=1, class_id=1,
            x_mm=50.0, y_mm=100.0,
            angle_deg=0.0, servo_correction_deg=0.0
        )

        # Corrupt a payload byte
        corrupted = bytearray(pkt)
        corrupted[5] ^= 0xFF  # Flip class_id byte
        result = decode_detection_packet(bytes(corrupted))
        self.assertIsNone(result, "Corrupted packet should be rejected")

    def test_crc16_header_corruption_rejection(self):
        """Corrupt the header — should be rejected."""
        pkt = encode_detection_packet(obj_id=1, class_id=1, x_mm=0, y_mm=0, angle_deg=0, servo_correction_deg=0)
        corrupted = bytearray(pkt)
        corrupted[0] = 0x00
        result = decode_detection_packet(bytes(corrupted))
        self.assertIsNone(result, "Corrupted header should be rejected")

    def test_wrong_length_rejection(self):
        """Wrong-length data should be rejected."""
        self.assertIsNone(decode_detection_packet(b'\xAA\x55' + b'\x00' * 10))
        self.assertIsNone(decode_detection_packet(b''))

    def test_heartbeat_packet(self):
        """Verify heartbeat packet format."""
        pkt = encode_heartbeat_packet(seq=7)
        self.assertEqual(len(pkt), 18)

        decoded = decode_detection_packet(pkt)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded['msg_type'], 0x02)
        self.assertEqual(decoded['obj_id'], 7)

    def test_ascii_debug_format(self):
        """Verify human-readable ASCII debug output."""
        result = format_ascii_debug(1, 2, 100.5, 50.3, 45.0, 90.0)
        self.assertIn("OBJ:1", result)
        self.assertIn("CLS:2", result)
        self.assertIn("X:100.5", result)
        self.assertIn("Y:50.3", result)
        self.assertIn("ANG:45.0", result)
        self.assertIn("SRV:90.0", result)

    def test_pixel_to_mm_mapper(self):
        """Test pixel-to-conveyor coordinate conversion."""
        mapper = PixelToConveyorMapper(pixels_per_mm_x=5.0, pixels_per_mm_y=5.0)
        x_mm, y_mm = mapper.pixel_to_mm(250.0, 500.0)
        self.assertAlmostEqual(x_mm, 50.0)
        self.assertAlmostEqual(y_mm, 100.0)

    def test_crc16_deterministic(self):
        """CRC16 must be deterministic for the same input."""
        data = b'\xAA\x55\x01\x00\x00\x01\xE8\x03\xD0\x07\x00\x00\x00\x00'
        crc1 = crc16_ccitt(data)
        crc2 = crc16_ccitt(data)
        self.assertEqual(crc1, crc2, "CRC16 must be deterministic")
        self.assertIsInstance(crc1, int)
        self.assertGreaterEqual(crc1, 0)
        self.assertLessEqual(crc1, 0xFFFF)


# =============================================================================
# TEST 3: Conveyor Tracker Debounce
# =============================================================================
class TestTrackerDebounce(unittest.TestCase):
    """
    Simulate 60 frames of a single capacitor moving along the Y-axis.
    Assert that only 1 UART pick command is triggered.
    """

    def test_single_object_triggers_once(self):
        """
        A single capacitor moves from y=50 to y=530 over 60 frames
        in a 640x480 camera frame. Inspection line at 50% (y=240).
        Exactly 1 trigger should fire.
        """
        tracker = ConveyorTracker(
            max_disappeared=15,
            max_distance=80.0,
            inspection_line_y=0.5
        )

        frame_height = 480
        total_triggers = 0

        for frame_idx in range(60):
            # Object moves from top to bottom
            y_center = 50 + frame_idx * 8  # Moves 8px per frame
            x_center = 320  # Stays centered horizontally

            bbox = (x_center - 30, y_center - 20, x_center + 30, y_center + 20)

            detections = [{
                'bbox': bbox,
                'class_id': 1,
                'class_name': 'Elco_Benar',
                'conf': 0.95,
                'roi': np.zeros((40, 60, 3), dtype=np.uint8),
                'angle': 0.0,
                'correction': 0.0,
            }]

            triggered = tracker.update(detections, frame_height)
            total_triggers += len(triggered)

        self.assertEqual(
            total_triggers, 1,
            f"Expected exactly 1 trigger, got {total_triggers}. "
            "Tracker is not debouncing correctly."
        )

    def test_two_objects_trigger_separately(self):
        """
        Two objects moving along the conveyor should each trigger exactly once.
        """
        tracker = ConveyorTracker(
            max_disappeared=15,
            max_distance=80.0,
            inspection_line_y=0.5
        )

        frame_height = 480
        total_triggers = 0

        for frame_idx in range(80):
            detections = []

            # Object 1: starts earlier
            y1 = 50 + frame_idx * 6
            if 0 <= y1 <= 470:
                detections.append({
                    'bbox': (100, y1 - 20, 160, y1 + 20),
                    'class_id': 1,
                    'class_name': 'Elco_Benar',
                    'conf': 0.92,
                    'roi': np.zeros((40, 60, 3), dtype=np.uint8),
                    'angle': 45.0,
                    'correction': 45.0,
                })

            # Object 2: starts 20 frames later (offset in Y)
            y2 = 50 + max(0, frame_idx - 20) * 6
            if frame_idx >= 20 and 0 <= y2 <= 470:
                detections.append({
                    'bbox': (400, y2 - 20, 460, y2 + 20),
                    'class_id': 2,
                    'class_name': 'Elco_Salah',
                    'conf': 0.88,
                    'roi': np.zeros((40, 60, 3), dtype=np.uint8),
                    'angle': 180.0,
                    'correction': 180.0,
                })

            triggered = tracker.update(detections, frame_height)
            total_triggers += len(triggered)

        self.assertEqual(
            total_triggers, 2,
            f"Expected 2 triggers (one per object), got {total_triggers}"
        )

    def test_no_detections_no_triggers(self):
        """Empty frames should produce no triggers."""
        tracker = ConveyorTracker()
        for _ in range(30):
            triggered = tracker.update([], 480)
            self.assertEqual(len(triggered), 0)

    def test_object_above_line_not_triggered(self):
        """Object that stays above inspection line should not trigger."""
        tracker = ConveyorTracker(inspection_line_y=0.8)  # Line at 80%
        frame_height = 480

        total_triggers = 0
        for frame_idx in range(30):
            # Object stays in top 30% of frame
            y = 50 + frame_idx * 2  # max y = 108, which is < 384 (0.8 * 480)
            detections = [{
                'bbox': (300, y - 15, 340, y + 15),
                'class_id': 1,
                'class_name': 'Elco_Benar',
                'conf': 0.9,
                'roi': np.zeros((30, 40, 3), dtype=np.uint8),
                'angle': 0.0,
                'correction': 0.0,
            }]
            triggered = tracker.update(detections, frame_height)
            total_triggers += len(triggered)

        self.assertEqual(total_triggers, 0,
                         "Object above inspection line should not trigger")

    def test_reset_clears_tracks(self):
        """After reset, tracker should have no tracks."""
        tracker = ConveyorTracker()
        detections = [{
            'bbox': (100, 200, 150, 250),
            'class_id': 1,
            'class_name': 'Elco_Benar',
            'conf': 0.9,
            'roi': np.zeros((50, 50, 3), dtype=np.uint8),
            'angle': 0.0,
            'correction': 0.0,
        }]
        tracker.update(detections, 480)
        self.assertGreater(len(tracker.tracks), 0)

        tracker.reset()
        self.assertEqual(len(tracker.tracks), 0)
        self.assertEqual(tracker.next_track_id, 1)


# =============================================================================
# TEST 4: Launcher Batch Files — No Hardcoded User Paths
# =============================================================================
class TestLauncherPortability(unittest.TestCase):
    """Verify .bat launchers have no hardcoded user paths."""

    LAUNCHER_DIR = ROOT_DIR / "06_Launchers"
    HARDCODED_PATTERNS = [
        r"C:\\Users\\HUSEN",
        r"C:/Users/HUSEN",
        r"\.conda\\envs\\",
        r"\.conda/envs/",
    ]

    def _check_file(self, filepath: Path):
        """Check a single file for hardcoded paths."""
        if not filepath.exists():
            self.skipTest(f"File not found: {filepath}")

        content = filepath.read_text(encoding='utf-8', errors='replace')
        for pattern in self.HARDCODED_PATTERNS:
            matches = re.findall(pattern, content, re.IGNORECASE)
            self.assertEqual(
                len(matches), 0,
                f"{filepath.name} contains hardcoded path matching '{pattern}': {matches}"
            )

    def test_launcher_1_uji_offline(self):
        self._check_file(self.LAUNCHER_DIR / "1_UJI_OFFLINE.bat")

    def test_launcher_2_uji_single_image(self):
        self._check_file(self.LAUNCHER_DIR / "2_UJI_SINGLE_IMAGE.bat")

    def test_launcher_3_uji_realtime(self):
        self._check_file(self.LAUNCHER_DIR / "3_UJI_REALTIME.bat")

    def test_launcher_4_evaluasi_model(self):
        self._check_file(self.LAUNCHER_DIR / "4_EVALUASI_MODEL.bat")

    def test_launcher_5_test_uart(self):
        self._check_file(self.LAUNCHER_DIR / "5_TEST_UART_ESP32.bat")

    def test_launcher_6_uji_video(self):
        self._check_file(self.LAUNCHER_DIR / "6_UJI_VIDEO.bat")

    def test_root_test_offline(self):
        self._check_file(ROOT_DIR / "TEST_OFFLINE.bat")


# =============================================================================
# TEST 5: Source Code Quality Checks
# =============================================================================
class TestSourceCodeQuality(unittest.TestCase):
    """Verify source code fixes at the code level."""

    def test_angle_calculator_no_class_name_in_angle_logic(self):
        """
        The angle_calculator.py evaluasi_qc_dan_servo function must NOT
        use class_name to determine the physical angle value.
        Specifically: no 'if "benar" in nama_lower' for setting sudut_deteksi.
        """
        src_path = ROOT_DIR / "04_Source_Code" / "utils" / "angle_calculator.py"
        content = src_path.read_text(encoding='utf-8')

        # Find the evaluasi_qc_dan_servo function body
        func_start = content.find("def evaluasi_qc_dan_servo")
        self.assertGreater(func_start, 0, "Function not found")

        # Find next function definition (end of evaluasi_qc_dan_servo)
        func_end = content.find("\ndef ", func_start + 10)
        if func_end == -1:
            func_end = len(content)

        func_body = content[func_start:func_end]

        # The old code had: sudut_deteksi = 0.0 if "benar" in nama_lower else 180.0
        # This must NOT exist anymore
        self.assertNotIn(
            'sudut_deteksi = 0.0 if "benar"',
            func_body,
            "Class name fallback for angle still exists in evaluasi_qc_dan_servo!"
        )
        self.assertNotIn(
            "sudut_deteksi = 0.0 if 'benar'",
            func_body,
            "Class name fallback for angle still exists!"
        )

    def test_firmware_no_nvs_writes_in_loop(self):
        """
        firmware_esp32.ino must NOT contain preferences.putString or
        simpanEntryNVS calls in the real-time loop.
        """
        fw_path = ROOT_DIR / "04_Source_Code" / "firmware_esp32" / "firmware_esp32.ino"
        content = fw_path.read_text(encoding='utf-8')

        # These destructive NVS write functions should be completely removed
        self.assertNotIn(
            "preferences.putString",
            content,
            "NVS putString still exists in firmware — will destroy flash memory!"
        )
        self.assertNotIn(
            "simpanEntryNVS",
            content,
            "simpanEntryNVS function still exists — must be removed!"
        )
        self.assertNotIn(
            "simpanCounterNVS",
            content,
            "simpanCounterNVS function still exists — must be removed!"
        )

    def test_firmware_no_blocking_delay(self):
        """
        firmware_esp32.ino must NOT contain blocking delay() calls
        in the loop() or prosesPaket functions.
        """
        fw_path = ROOT_DIR / "04_Source_Code" / "firmware_esp32" / "firmware_esp32.ino"
        content = fw_path.read_text(encoding='utf-8')

        # Find all delay() calls
        delay_pattern = re.compile(r'\bdelay\s*\(\s*\d+\s*\)')
        matches = delay_pattern.findall(content)

        self.assertEqual(
            len(matches), 0,
            f"Blocking delay() calls found in firmware: {matches}. "
            "Replace with non-blocking millis() state machine."
        )

    def test_train_py_valid_model(self):
        """1_train.py must reference a valid Ultralytics model, not yolo26m.pt."""
        train_path = ROOT_DIR / "04_Source_Code" / "1_train.py"
        content = train_path.read_text(encoding='utf-8')

        self.assertNotIn(
            "yolo26m.pt",
            content,
            "1_train.py still references nonexistent yolo26m.pt model"
        )

        # Must have a valid model reference
        valid_models = ["yolov8n.pt", "yolov8s.pt", "yolov8m.pt", "yolo11n.pt"]
        has_valid = any(m in content for m in valid_models)
        self.assertTrue(
            has_valid,
            f"1_train.py does not reference any valid model. Found none of: {valid_models}"
        )


# =============================================================================
# TEST 6: Integration Smoke Tests
# =============================================================================
class TestIntegration(unittest.TestCase):
    """Quick integration smoke tests combining multiple modules."""

    def test_full_pipeline_synthetic(self):
        """
        Full pipeline: generate ROI → compute angle → build UART packet → decode.
        """
        for target_angle in [0.0, 45.0, 90.0, 180.0]:
            roi = make_capacitor_roi(target_angle, size=120)

            # Run QC evaluation
            sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_msg = \
                evaluasi_qc_dan_servo(1, "Elco_Benar", roi)

            # Build binary UART packet
            mapper = PixelToConveyorMapper(pixels_per_mm_x=5.0, pixels_per_mm_y=5.0)
            x_mm, y_mm = mapper.pixel_to_mm(320.0, 240.0)

            pkt = encode_detection_packet(
                obj_id=1, class_id=1,
                x_mm=x_mm, y_mm=y_mm,
                angle_deg=sudut_act,
                servo_correction_deg=kor_servo
            )

            # Decode and verify
            decoded = decode_detection_packet(pkt)
            self.assertIsNotNone(decoded, f"Failed to decode packet for angle {target_angle}")
            self.assertAlmostEqual(decoded['angle_deg'], sudut_act, places=0)


# =============================================================================
# MAIN — Run all tests
# =============================================================================
if __name__ == "__main__":
    print("=" * 70)
    print("  AUTOMATED VERIFICATION SUITE — VisionSystem Remediation v2.0")
    print("  Tugas Akhir: QC Kapasitor | Husein Alhamid (4212301035)")
    print("=" * 70)
    print()

    # Run with verbosity
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    suite.addTests(loader.loadTestsFromTestCase(TestContinuousAngle))
    suite.addTests(loader.loadTestsFromTestCase(TestUARTProtocol))
    suite.addTests(loader.loadTestsFromTestCase(TestTrackerDebounce))
    suite.addTests(loader.loadTestsFromTestCase(TestLauncherPortability))
    suite.addTests(loader.loadTestsFromTestCase(TestSourceCodeQuality))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print()
    if result.wasSuccessful():
        print("=" * 70)
        print("  ✓ ALL TESTS PASSED — VisionSystem remediation verified!")
        print("=" * 70)
    else:
        print("=" * 70)
        print("  ✗ SOME TESTS FAILED — Review output above")
        print("=" * 70)

    sys.exit(0 if result.wasSuccessful() else 1)
