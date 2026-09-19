"""
===============================================================================
  6_test_video.py — PENGUJIAN DETEKSI QC KAPASITOR VIA FILE VIDEO + UART ESP32
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
===============================================================================

  Deskripsi:
  Script ini membaca file video (.mp4, .avi, .mov, .mkv), menjalankan inferensi
  YOLO frame-per-frame, mengirim data koreksi ke ESP32 via UART, dan menampilkan
  visualisasi real-time serta rekap CSV.

  Output UART akan muncul di Serial Monitor Arduino IDE (jika ESP32 terhubung).

  Cara menjalankan:
    python 6_test_video.py --video "path/ke/video.mp4"
    python 6_test_video.py --video "path/ke/video.mp4" --uart-port COM3
    python 6_test_video.py --video "path/ke/video.mp4" --no-uart
    python 6_test_video.py --video "path/ke/video.mp4" --save-video --speed 0.5

===============================================================================
"""

import os
import sys
import io
import time
import csv
import argparse
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

import cv2
import numpy as np
from ultralytics import YOLO

# Import helper modul
ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.append(str(ROOT_DIR / "04_Source_Code"))

from utils.angle_calculator import evaluasi_qc_dan_servo, gambar_anotasi
from utils.uart_handler import KoneksiUART

DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"
DEFAULT_OUTPUT_DIR = ROOT_DIR / "05_Hasil_Pengujian" / "video_test_results"

# Ekstensi video yang didukung
VIDEO_EXTENSIONS = {'.mp4', '.avi', '.mov', '.mkv', '.wmv', '.flv', '.webm'}


def format_waktu(detik: float) -> str:
    """Format detik ke MM:SS.mmm"""
    m = int(detik // 60)
    s = detik % 60
    return f"{m:02d}:{s:06.3f}"


def run_video_test(args):
    # ─── Validasi input video ───
    video_path = Path(args.video)
    if not video_path.exists():
        print(f"[ERROR] File video tidak ditemukan: {video_path}")
        sys.exit(1)
    if video_path.suffix.lower() not in VIDEO_EXTENSIONS:
        print(f"[ERROR] Format file tidak didukung: {video_path.suffix}")
        print(f"        Format yang didukung: {', '.join(sorted(VIDEO_EXTENSIONS))}")
        sys.exit(1)

    # ─── Validasi model ───
    model_path = Path(args.weights)
    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)

    # ─── Header informasi ───
    print("=" * 75)
    print("  PENGUJIAN DETEKSI QC KAPASITOR VIA VIDEO — HUSEIN ALHAMID")
    print("=" * 75)
    print(f"  File Video       : {video_path.name}")
    print(f"  Model Weights    : {model_path.name}")
    print(f"  Confidence       : {args.conf}")
    print(f"  UART Port        : {args.uart_port if not args.no_uart else 'DISABLED (mode offline)'}")
    print(f"  Kecepatan        : {args.speed}x")
    print(f"  Simpan Video     : {'Ya' if args.save_video else 'Tidak'}")
    print("=" * 75)

    # ─── Load model YOLO ───
    model = YOLO(str(model_path))
    class_names = model.names

    # ─── Buka video ───
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka file video: {video_path}")
        sys.exit(1)

    # Ambil properti video
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    durasi_video = total_frames / video_fps if video_fps > 0 else 0

    print(f"\n  Resolusi Video   : {video_width} x {video_height}")
    print(f"  FPS Video        : {video_fps:.1f}")
    print(f"  Total Frame      : {total_frames}")
    print(f"  Durasi Video     : {format_waktu(durasi_video)}")
    print("=" * 75)

    # ─── Setup output ───
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_file = out_dir / f"rekap_video_{video_path.stem}.csv"
    screenshot_dir = out_dir / "screenshots"
    screenshot_dir.mkdir(parents=True, exist_ok=True)

    # Video writer (opsional)
    video_writer = None
    if args.save_video:
        out_video_path = out_dir / f"result_{video_path.stem}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(
            str(out_video_path), fourcc, video_fps,
            (video_width, video_height)
        )
        print(f"  Output Video     : {out_video_path}")

    # ─── Inisialisasi UART ke ESP32 ───
    uart = None
    if not args.no_uart:
        uart = KoneksiUART(port=args.uart_port, baudrate=args.baudrate)
        if uart.is_connected:
            print(f"\n[✓] ESP32 terhubung pada {uart.port} @ {args.baudrate} bps")
            print("    Data koreksi akan dikirim ke ESP32.")
            print("    Output ESP32 akan muncul di Serial Monitor Arduino IDE.")
        else:
            print(f"\n[!] ESP32 tidak terhubung. Berjalan dalam mode SIMULASI.")
            print("    Data UART akan ditampilkan di terminal ini saja.")
    else:
        print(f"\n[i] Mode OFFLINE (tanpa UART). Hanya inferensi dan visualisasi.")

    # ─── CSV header ───
    csv_headers = [
        "No", "Frame", "Timestamp_Video", "Waktu_Inferensi_ms", "ID_Objek",
        "Kelas_Prediksi", "Confidence", "BBox_xyxy",
        "Sudut_Aktual_deg", "Sudut_Ref_deg", "Deviasi_deg",
        "Koreksi_Servo_deg", "Arah_Servo", "Status_QC",
        "Aksi_Sistem", "Perintah_UART", "ESP32_ACK"
    ]
    hasil_rekap = []
    total_deteksi = 0
    frame_count = 0
    paused = False

    # Hitung delay antar-frame untuk kontrol playback
    delay_base_ms = int(1000.0 / video_fps) if video_fps > 0 else 33

    print(f"\n[✓] Memulai pengujian video... (Total: {total_frames} frame)")
    print("    Kontrol: [SPASI] Pause/Resume | [Q] Keluar | [S] Screenshot")
    print("    " + "-" * 71)
    print(f"    {'Frame':<8} | {'Waktu':<12} | {'Kelas':<22} | {'Conf':<6} | {'Status QC':<25} | {'UART':<20}")
    print("    " + "-" * 71)

    t_start_global = time.time()

    try:
        while True:
            # ─── Handle pause ───
            if paused:
                key = cv2.waitKey(50) & 0xFF
                if key == ord(' '):
                    paused = False
                    print("    [▶] RESUMED")
                elif key == ord('q'):
                    print("    [✕] Dihentikan oleh pengguna (saat pause).")
                    break
                continue

            # ─── Baca frame ───
            ret, frame = cap.read()
            if not ret:
                print("\n    [✓] Seluruh frame video telah diproses.")
                break

            frame_count += 1
            timestamp_video = frame_count / video_fps if video_fps > 0 else 0.0

            # ─── Inferensi YOLO ───
            t0 = time.perf_counter()
            results = model.predict(frame, conf=args.conf, iou=args.iou, verbose=False)
            t_inf_ms = (time.perf_counter() - t0) * 1000.0

            annotated_frame = frame.copy()
            frame_has_detection = False

            for r in results:
                for idx_b, box in enumerate(r.boxes):
                    frame_has_detection = True
                    total_deteksi += 1
                    cls_id = int(box.cls[0].item())
                    cls_name = class_names.get(cls_id, str(cls_id))
                    conf_val = float(box.conf[0].item())
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)

                    roi = frame[xyxy[1]:xyxy[3], xyxy[0]:xyxy[2]]
                    sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_msg = evaluasi_qc_dan_servo(
                        cls_id, cls_name, roi
                    )

                    # ─── Kirim data ke ESP32 via UART ───
                    esp32_ack = ""
                    if uart:
                        berhasil = uart.kirim(uart_msg)
                        if berhasil:
                            print(f"    [UART -> ESP32] Frame {frame_count:>5}: {uart_msg.strip()}")

                        # Baca balasan ACK dari ESP32
                        time.sleep(0.05)  # Jeda kecil agar ESP32 sempat merespon
                        ack = uart.baca_respon()
                        if ack:
                            esp32_ack = ack
                            print(f"    [ESP32 -> PC  ] Balasan: {ack}")

                    # ─── Rekap CSV ───
                    hasil_rekap.append([
                        total_deteksi, frame_count,
                        format_waktu(timestamp_video),
                        f"{t_inf_ms:.1f}", idx_b + 1,
                        cls_name, f"{conf_val:.3f}",
                        f"[{xyxy[0]},{xyxy[1]},{xyxy[2]},{xyxy[3]}]",
                        f"{sudut_act:.1f}", f"{s_ref:.1f}", f"{dev:+.1f}",
                        f"{kor_servo:+.1f}", arah, st_qc, aksi,
                        uart_msg.strip(), esp32_ack
                    ])

                    # ─── Gambar anotasi ───
                    annotated_frame = gambar_anotasi(
                        annotated_frame, xyxy, cls_name, conf_val,
                        sudut_act, dev, kor_servo, st_qc, aksi, sudut_ref=s_ref
                    )

                    # ─── Print ke terminal ───
                    print(f"    {frame_count:<8} | {format_waktu(timestamp_video):<12} | "
                          f"{cls_name:<22} | {conf_val*100:>5.1f}% | {st_qc:<25} | {uart_msg.strip():<20}")

            # ─── Status Header Bar pada frame ───
            h, w = frame.shape[:2]
            cv2.rectangle(annotated_frame, (0, 0), (w, 40), (15, 15, 15), -1)

            # Progress bar
            progress = frame_count / total_frames if total_frames > 0 else 0
            bar_width = int(w * progress)
            cv2.rectangle(annotated_frame, (0, 36), (bar_width, 40), (34, 197, 94), -1)

            # UART status
            uart_status = "UART: OFF"
            c_uart = (120, 120, 120)
            if uart:
                if uart.is_connected:
                    uart_status = f"ESP32: ON ({uart.port})"
                    c_uart = (34, 197, 94)
                else:
                    uart_status = "ESP32: SIMULASI"
                    c_uart = (0, 165, 255)

            inf_fps = 1000.0 / t_inf_ms if t_inf_ms > 0 else 0
            header = (f"VIDEO TEST | Frame: {frame_count}/{total_frames} | "
                      f"{format_waktu(timestamp_video)} | "
                      f"Inf: {t_inf_ms:.0f}ms ({inf_fps:.0f}fps) | "
                      f"{uart_status} | Det: {total_deteksi}")
            cv2.putText(annotated_frame, header,
                        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.48, c_uart, 1, cv2.LINE_AA)

            # ─── Tampilkan frame ───
            cv2.imshow("Pengujian Video QC Kapasitor (Husein TA)", annotated_frame)

            # ─── Simpan frame ke video output ───
            if video_writer:
                video_writer.write(annotated_frame)

            # ─── Kontrol playback & keyboard ───
            effective_delay = max(1, int(delay_base_ms / args.speed))
            key = cv2.waitKey(effective_delay) & 0xFF

            if key == ord('q'):
                print(f"\n    [✕] Dihentikan oleh pengguna pada frame {frame_count}/{total_frames}.")
                break
            elif key == ord(' '):
                paused = True
                print(f"    [⏸] PAUSED pada frame {frame_count} ({format_waktu(timestamp_video)})")
            elif key == ord('s'):
                ss_path = screenshot_dir / f"ss_frame{frame_count}_{int(time.time())}.jpg"
                cv2.imwrite(str(ss_path), annotated_frame)
                print(f"    [📸] Screenshot tersimpan: {ss_path.name}")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        if video_writer:
            video_writer.release()
        if uart:
            uart.tutup()

    # ─── Simpan CSV rekap ───
    with open(csv_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)
        writer.writerows(hasil_rekap)

    durasi_total = time.time() - t_start_global

    # ─── Ringkasan Akhir ───
    print("\n" + "=" * 75)
    print("  PENGUJIAN VIDEO SELESAI")
    print("=" * 75)
    print(f"  File Video         : {video_path.name}")
    print(f"  Frame Diproses     : {frame_count} / {total_frames}")
    print(f"  Total Deteksi      : {total_deteksi} objek")
    print(f"  Durasi Pengujian   : {durasi_total:.2f} detik")
    print(f"  Kecepatan Rata²    : {frame_count / durasi_total:.1f} frame/detik" if durasi_total > 0 else "")
    print(f"  Laporan CSV        : {csv_file}")
    if args.save_video:
        out_video_path = out_dir / f"result_{video_path.stem}.mp4"
        print(f"  Video Beranotasi   : {out_video_path}")
    if uart and uart.is_connected:
        print(f"  UART ESP32         : Data terkirim ke {uart.port}")
        print(f"                       Cek Serial Monitor Arduino IDE untuk log ESP32")
    print("=" * 75 + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Pengujian Deteksi QC Kapasitor via File Video + UART ESP32"
    )
    parser.add_argument('--video', type=str, required=True,
                        help='Path ke file video (.mp4, .avi, .mov, .mkv)')
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL),
                        help='Path model .pt (default: best.pt)')
    parser.add_argument('--conf', type=float, default=0.40,
                        help='Confidence threshold (default: 0.40)')
    parser.add_argument('--iou', type=float, default=0.45,
                        help='NMS IoU threshold (default: 0.45)')
    parser.add_argument('--no-uart', action='store_true',
                        help='Nonaktifkan komunikasi UART ke ESP32 (mode offline)')
    parser.add_argument('--uart-port', type=str, default="AUTO",
                        help='Port UART serial ESP32 (misal AUTO, COM3, COM4)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baudrate UART (default: 115200)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Multiplier kecepatan playback (default: 1.0, lambat: 0.5)')
    parser.add_argument('--save-video', action='store_true',
                        help='Simpan video output beranotasi (.mp4)')
    parser.add_argument('--output', type=str, default=str(DEFAULT_OUTPUT_DIR),
                        help='Folder output hasil pengujian')

    args = parser.parse_args()
    run_video_test(args)
