"""
===============================================================================
  4_detect_realtime.py — SISTEM DETEKSI REAL-TIME + KOMUNIKASI UART STM32
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
===============================================================================

  REVISION v2.0 (Remediation):
  - Integrated ConveyorTracker for spatial object tracking and debounce
  - Upgraded to binary framed UART protocol with CRC16 checksum
  - Added pixel-to-mm coordinate mapping for STM32 Cartesian manipulator
  - Each capacitor triggers EXACTLY ONE UART command when crossing inspection line
  - Prevents frame-rate UART flooding
===============================================================================
"""

import os
import sys
import io
import time
import argparse
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

import cv2
import numpy as np
from ultralytics import YOLO

ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.append(str(ROOT_DIR / "04_Source_Code"))

from utils.angle_calculator import evaluasi_qc_dan_servo, gambar_anotasi
from utils.uart_handler import KoneksiUART
from utils.uart_protocol import (
    encode_detection_packet, format_ascii_debug, PixelToConveyorMapper
)
from utils.conveyor_tracker import ConveyorTracker

DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"

def detect_realtime(args):
    model_path = Path(args.weights)
    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)

    print("=" * 70)
    print("  DETEKSI REAL-TIME QC KAPASITOR v2.0 — HUSEIN ALHAMID")
    print("=" * 70)
    print(f"  Model Weights : {model_path.name}")
    print(f"  Camera Source : {args.source}")
    print(f"  UART Port     : {args.uart_port if not args.no_uart else 'DISABLED'}")
    print(f"  Protocol      : Binary Framed (CRC16) + ASCII Debug")
    print(f"  Tracking      : Enabled (debounce, inspection line @ {args.inspection_line:.0%})")
    print("=" * 70)

    model = YOLO(str(model_path))
    class_names = model.names

    # Buka Kamera
    cam_src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(cam_src)
    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka sumber kamera: {cam_src}")
        sys.exit(1)

    # Inisialisasi UART ke STM32
    uart = KoneksiUART(port=args.uart_port, baudrate=args.baudrate) if not args.no_uart else None

    # Inisialisasi Pixel-to-Conveyor Mapper
    coord_mapper = PixelToConveyorMapper(
        pixels_per_mm_x=args.px_per_mm_x,
        pixels_per_mm_y=args.px_per_mm_y
    )

    # Inisialisasi Conveyor Tracker
    tracker = ConveyorTracker(
        max_disappeared=args.max_disappeared,
        max_distance=args.max_distance,
        inspection_line_y=args.inspection_line
    )

    print("\n[✓] Kamera aktif! Tekan 'Q' untuk keluar, 'S' untuk screenshot.")
    fps_history = []

    try:
        while True:
            t0 = time.perf_counter()
            ret, frame = cap.read()
            if not ret:
                break

            frame_h, frame_w = frame.shape[:2]
            results = model.predict(frame, conf=args.conf, iou=args.iou, verbose=False)
            annotated_frame = frame.copy()

            # ─── Collect all detections this frame ───
            frame_detections = []
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0].item())
                    cls_name = class_names.get(cls_id, str(cls_id))
                    conf_val = float(box.conf[0].item())
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)

                    roi = frame[xyxy[1]:xyxy[3], xyxy[0]:xyxy[2]]
                    sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_msg_legacy = evaluasi_qc_dan_servo(
                        cls_id, cls_name, roi
                    )

                    frame_detections.append({
                        'bbox': tuple(xyxy),
                        'class_id': cls_id,
                        'class_name': cls_name,
                        'conf': conf_val,
                        'roi': roi,
                        'angle': sudut_act,
                        'correction': kor_servo,
                        'sudut_ref': s_ref,
                        'deviasi': dev,
                        'arah': arah,
                        'status_qc': st_qc,
                        'aksi': aksi,
                    })

                    # Draw annotation for ALL detections (visual feedback)
                    annotated_frame = gambar_anotasi(
                        annotated_frame, xyxy, cls_name, conf_val,
                        sudut_act, dev, kor_servo, st_qc, aksi, sudut_ref=s_ref
                    )

            # ─── Update tracker and send UART only for newly triggered objects ───
            triggered = tracker.update(frame_detections, frame_h)

            for obj in triggered:
                # Convert pixel centroid to conveyor mm coordinates
                cx, cy = obj['centroid']
                x_mm, y_mm = coord_mapper.pixel_to_mm(cx, cy)

                if uart:
                    # Send binary framed packet with CRC16
                    pkt = encode_detection_packet(
                        obj_id=obj['track_id'],
                        class_id=obj['class_id'],
                        x_mm=x_mm,
                        y_mm=y_mm,
                        angle_deg=obj['angle'],
                        servo_correction_deg=obj['correction']
                    )
                    berhasil = uart.kirim(pkt.decode('latin-1') if isinstance(pkt, bytes) else pkt, force=True)
                    if berhasil:
                        debug_str = format_ascii_debug(
                            obj['track_id'], obj['class_id'],
                            x_mm, y_mm, obj['angle'], obj['correction']
                        )
                        print(f"[UART → STM32] Track #{obj['track_id']}: {debug_str.strip()}")

                    # Read ACK
                    ack = uart.baca_respon()
                    if ack:
                        print(f"[STM32 → PC] ACK: {ack}")

            # ─── Draw inspection line on frame ───
            insp_y = int(args.inspection_line * frame_h)
            cv2.line(annotated_frame, (0, insp_y), (frame_w, insp_y), (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, "INSPECTION LINE", (10, insp_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

            fps = 1.0 / (time.perf_counter() - t0 + 1e-9)
            fps_history.append(fps)
            avg_fps = np.mean(fps_history[-30:])

            # Status Header Bar
            h, w = frame.shape[:2]
            cv2.rectangle(annotated_frame, (0, 0), (w, 35), (15, 15, 15), -1)

            uart_status = "UART: OFF"
            c_uart = (120, 120, 120)
            if uart:
                if uart.is_connected:
                    uart_status = f"STM32: ON ({uart.port})"
                    c_uart = (34, 197, 94)  # Hijau
                else:
                    uart_status = "STM32: SIMULASI"
                    c_uart = (0, 165, 255)  # Oranye

            track_count = len(tracker.tracks)
            header_text = f"QC Kapasitor v2.0 | FPS: {avg_fps:.1f} | {uart_status} | Tracks: {track_count} | [Q] Quit [S] SS"
            cv2.putText(annotated_frame, header_text,
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, c_uart, 1, cv2.LINE_AA)

            cv2.imshow("QC Kapasitor Real-Time v2.0 (Husein TA)", annotated_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                ss_path = ROOT_DIR / "05_Hasil_Pengujian" / f"screenshot_{int(time.time())}.jpg"
                cv2.imwrite(str(ss_path), annotated_frame)
                print(f"[✓] Screenshot tersimpan: {ss_path}")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        if uart:
            uart.tutup()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deteksi Real-Time QC Kapasitor + UART STM32 v2.0")
    parser.add_argument('--source', type=str, default="0", help='Index kamera (0/1/2) atau RTSP/Video path')
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL), help='Path model .pt')
    parser.add_argument('--conf', type=float, default=0.40, help='Confidence threshold')
    parser.add_argument('--iou', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--no-uart', action='store_true', help='Nonaktifkan komunikasi UART ke STM32')
    parser.add_argument('--uart-port', type=str, default="AUTO", help='Port UART serial (misal AUTO, COM3, COM4)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate UART (default: 115200)')
    # Conveyor tracker parameters
    parser.add_argument('--inspection-line', type=float, default=0.5, help='Inspection line Y fraction (0.0=top, 1.0=bottom)')
    parser.add_argument('--max-disappeared', type=int, default=15, help='Max frames before dropping track')
    parser.add_argument('--max-distance', type=float, default=80.0, help='Max centroid matching distance (pixels)')
    # Coordinate mapping parameters
    parser.add_argument('--px-per-mm-x', type=float, default=5.0, help='Pixels per mm on X axis')
    parser.add_argument('--px-per-mm-y', type=float, default=5.0, help='Pixels per mm on Y axis')
    args = parser.parse_args()
    detect_realtime(args)
