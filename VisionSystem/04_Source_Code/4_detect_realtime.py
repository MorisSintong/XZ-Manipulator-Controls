"""
===============================================================================
  4_detect_realtime.py — SISTEM DETEKSI REAL-TIME + KOMUNIKASI UART STM32
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)

  REMEDIATED:
  - Integrated ConveyorTracker for spatial object tracking & debounce
  - Uses binary framed UART protocol with CRC16 checksum
  - Pixel-to-mm coordinate mapping via PixelToConveyorMapper
  - Each capacitor generates exactly 1 pick command (no UART flooding)
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
    encode_detection_packet, format_debug_ascii, PixelToConveyorMapper
)
from utils.conveyor_tracker import ConveyorTracker, Detection

DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"


def detect_realtime(args):
    model_path = Path(args.weights)
    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)

    print("=" * 70)
    print("  DETEKSI REAL-TIME QC KAPASITOR — HUSEIN ALHAMID")
    print("  (REMEDIATED: Tracker + Binary UART Protocol)")
    print("=" * 70)
    print(f"  Model Weights : {model_path.name}")
    print(f"  Camera Source : {args.source}")
    print(f"  UART Port     : {args.uart_port if not args.no_uart else 'DISABLED'}")
    print(f"  UART Mode     : {'Binary Framed (CRC16)' if not args.ascii_uart else 'ASCII Debug'}")
    print(f"  Inspection Y  : {args.inspection_line}")
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

    # Inisialisasi Conveyor Tracker (prevents UART flooding)
    tracker = ConveyorTracker(
        max_disappeared=args.max_disappeared,
        distance_threshold=args.track_distance,
        inspection_line_y=args.inspection_line,
        min_frames_before_eval=args.min_track_frames,
    )

    # Inisialisasi Pixel-to-mm Mapper
    coord_mapper = PixelToConveyorMapper(
        scale_x=args.scale_x,
        scale_y=args.scale_y,
    )

    print("\n[✓] Kamera aktif! Tekan 'Q' untuk keluar, 'S' untuk screenshot.")
    fps_history = []
    total_picks_sent = 0

    try:
        while True:
            t0 = time.perf_counter()
            ret, frame = cap.read()
            if not ret:
                break

            results = model.predict(frame, conf=args.conf, iou=args.iou, verbose=False)
            annotated_frame = frame.copy()

            # ─── Collect detections for tracker ───
            frame_detections = []

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0].item())
                    cls_name = class_names.get(cls_id, str(cls_id))
                    conf_val = float(box.conf[0].item())
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)

                    roi = frame[xyxy[1]:xyxy[3], xyxy[0]:xyxy[2]]
                    sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_data = evaluasi_qc_dan_servo(
                        cls_id, cls_name, roi
                    )

                    # Create Detection for tracker
                    det = Detection(
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=conf_val,
                        bbox=tuple(xyxy),
                        angle_deg=sudut_act,
                        correction_deg=kor_servo,
                        status_qc=st_qc,
                        aksi=aksi,
                    )
                    frame_detections.append(det)

                    # Draw annotation (visual feedback on every frame is OK)
                    annotated_frame = gambar_anotasi(
                        annotated_frame, xyxy, cls_name, conf_val,
                        sudut_act, dev, kor_servo, st_qc, aksi, sudut_ref=s_ref
                    )

            # ─── Update tracker with all detections ───
            active_tracks = tracker.update(frame_detections)

            # ─── Send UART only for newly-dispatched picks (debounced) ───
            pending_picks = tracker.get_pending_picks()
            for pick in pending_picks:
                # Convert pixel centroid to conveyor mm coordinates
                px, py = pick.centroid
                x_mm, y_mm = coord_mapper.pixel_to_mm(px, py)

                if uart:
                    if args.ascii_uart:
                        # ASCII debug mode
                        msg = format_debug_ascii(
                            obj_id=pick.track_id,
                            class_id=pick.best_class_id,
                            x_mm=x_mm,
                            y_mm=y_mm,
                            angle_deg=pick.best_angle_deg,
                            correction_deg=pick.best_correction_deg,
                        )
                        berhasil = uart.kirim(msg, force=True)
                    else:
                        # Binary framed packet with CRC16
                        packet = encode_detection_packet(
                            obj_id=pick.track_id,
                            class_id=pick.best_class_id,
                            x_mm=x_mm,
                            y_mm=y_mm,
                            angle_deg=pick.best_angle_deg,
                            correction_deg=pick.best_correction_deg,
                        )
                        berhasil = uart.kirim_paket(packet)

                    if berhasil:
                        total_picks_sent += 1
                        print(
                            f"[UART → STM32] Pick #{total_picks_sent} | "
                            f"Track={pick.track_id} CLS={pick.best_class_name} "
                            f"X={x_mm:.1f}mm Y={y_mm:.1f}mm "
                            f"ANG={pick.best_angle_deg:.1f}° "
                            f"COR={pick.best_correction_deg:.1f}°"
                        )

                # Baca balasan/ACK dari STM32 jika ada
                if uart:
                    ack = uart.baca_respon()
                    if ack:
                        print(f"[STM32 → UART] ACK: {ack}")

            fps = 1.0 / (time.perf_counter() - t0 + 1e-9)
            fps_history.append(fps)
            avg_fps = np.mean(fps_history[-30:])

            # ─── Draw inspection line ───
            h, w = frame.shape[:2]
            cv2.line(annotated_frame, (0, args.inspection_line), (w, args.inspection_line),
                     (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, "Inspection Line",
                        (10, args.inspection_line - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

            # Status Header Bar
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

            header_text = (
                f"YOLO26 QC | FPS: {avg_fps:.1f} | {uart_status} | "
                f"Tracks: {len(active_tracks)} | Picks: {total_picks_sent} | "
                f"[Q] Exit [S] SS"
            )
            cv2.putText(annotated_frame, header_text,
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c_uart, 1, cv2.LINE_AA)

            cv2.imshow("QC Kapasitor Real-Time (Husein TA)", annotated_frame)
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
        print(f"\n[✓] Total pick commands sent: {total_picks_sent}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deteksi Real-Time QC Kapasitor + Komunikasi STM32 UART")
    parser.add_argument('--source', type=str, default="0", help='Index kamera (0/1/2) atau RTSP/Video path')
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL), help='Path model .pt')
    parser.add_argument('--conf', type=float, default=0.40, help='Confidence threshold')
    parser.add_argument('--iou', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--no-uart', action='store_true', help='Nonaktifkan komunikasi UART ke STM32')
    parser.add_argument('--uart-port', type=str, default="AUTO", help='Port UART serial STM32 (misal AUTO, COM3)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate UART (default: 115200)')
    parser.add_argument('--ascii-uart', action='store_true', help='Gunakan mode ASCII debug (bukan binary framed)')
    # Tracker parameters
    parser.add_argument('--inspection-line', type=int, default=240, help='Y pixel coordinate of inspection line')
    parser.add_argument('--max-disappeared', type=int, default=10, help='Max frames before track removal')
    parser.add_argument('--track-distance', type=float, default=80.0, help='Max centroid distance for matching (px)')
    parser.add_argument('--min-track-frames', type=int, default=3, help='Min frames before evaluation')
    # Coordinate mapping
    parser.add_argument('--scale-x', type=float, default=0.5, help='mm per pixel X (default: 0.5)')
    parser.add_argument('--scale-y', type=float, default=0.5, help='mm per pixel Y (default: 0.5)')
    args = parser.parse_args()
    detect_realtime(args)
