"""
===============================================================================
  4_detect_realtime.py — SISTEM DETEKSI REAL-TIME + KOMUNIKASI UART STM32
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
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

DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"

def detect_realtime(args):
    model_path = Path(args.weights)
    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)

    print("=" * 70)
    print("  DETEKSI REAL-TIME QC KAPASITOR — HUSEIN ALHAMID")
    print("=" * 70)
    print(f"  Model Weights : {model_path.name}")
    print(f"  Camera Source : {args.source}")
    print(f"  UART Port     : {args.uart_port if not args.no_uart else 'DISABLED'}")
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

    print("\n[✓] Kamera aktif! Tekan 'Q' untuk keluar, 'S' untuk screenshot.")
    fps_history = []

    try:
        while True:
            t0 = time.perf_counter()
            ret, frame = cap.read()
            if not ret:
                break

            results = model.predict(frame, conf=args.conf, iou=args.iou, verbose=False)
            annotated_frame = frame.copy()

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0].item())
                    cls_name = class_names.get(cls_id, str(cls_id))
                    conf_val = float(box.conf[0].item())
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)

                    roi = frame[xyxy[1]:xyxy[3], xyxy[0]:xyxy[2]]
                    sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_msg = evaluasi_qc_dan_servo(
                        cls_id, cls_name, roi
                    )

                    # Kirim data koreksi ke ESP32 via UART
                    if uart:
                        berhasil = uart.kirim(uart_msg)
                        if berhasil:
                            print(f"[UART -> ESP32] Data Terkirim: {uart_msg.strip()}")

                    # Baca balasan/ACK dari ESP32 jika ada
                    if uart:
                        ack = uart.baca_respon()
                        if ack:
                            print(f"[ESP32 -> UART] Balasan: {ack}")

                    annotated_frame = gambar_anotasi(
                        annotated_frame, xyxy, cls_name, conf_val,
                        sudut_act, dev, kor_servo, st_qc, aksi, sudut_ref=s_ref
                    )

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
                    uart_status = f"ESP32: ON ({uart.port})"
                    c_uart = (34, 197, 94)  # Hijau
                else:
                    uart_status = "ESP32: SIMULASI"
                    c_uart = (0, 165, 255)  # Oranye

            header_text = f"YOLO26 QC Kapasitor | FPS: {avg_fps:.1f} | {uart_status} | [Q] Keluar [S] SS"
            cv2.putText(annotated_frame, header_text,
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, c_uart, 1, cv2.LINE_AA)

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deteksi Real-Time QC Kapasitor + Komunikasi ESP32 UART")
    parser.add_argument('--source', type=str, default="0", help='Index kamera (0/1/2) atau RTSP/Video path')
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL), help='Path model .pt')
    parser.add_argument('--conf', type=float, default=0.40, help='Confidence threshold')
    parser.add_argument('--iou', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--no-uart', action='store_true', help='Nonaktifkan komunikasi UART ke ESP32')
    parser.add_argument('--uart-port', type=str, default="AUTO", help='Port UART serial ESP32 (misal AUTO, COM3, COM4)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate UART (default: 115200)')
    args = parser.parse_args()
    detect_realtime(args)
