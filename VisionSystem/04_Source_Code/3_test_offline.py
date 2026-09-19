"""
===============================================================================
  3_test_offline.py — PENGUJIAN DETEKSI & SUDUT OFFLINE (BATCH / VIDEO / GAMBAR)
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
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
from ultralytics import YOLO

# Import helper modul
ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.append(str(ROOT_DIR / "04_Source_Code"))

from utils.angle_calculator import evaluasi_qc_dan_servo, gambar_anotasi

DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"
DEFAULT_TEST_DIR = ROOT_DIR / "02_Dataset" / "Kapasitor_Labeling_1500.v1i.yolo26" / "test" / "images"
DEFAULT_OUTPUT_DIR = ROOT_DIR / "05_Hasil_Pengujian" / "offline_batch_results"

def run_test(args):
    model_path = Path(args.weights)
    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)

    print("=" * 75)
    print("  PENGUJIAN DETEKSI & SUDUT KAPASITOR OFFLINE (NON-REALTIME)")
    print("=" * 75)
    print(f"  Model Weights        : {model_path.name}")
    print(f"  Confidence Threshold : {args.conf}")
    print("=" * 75)

    model = YOLO(str(model_path))
    class_names = model.names

    source_path = Path(args.source)
    image_files = []
    is_video = False

    if source_path.is_file():
        ext = source_path.suffix.lower()
        if ext in ['.jpg', '.jpeg', '.png', '.bmp', '.webp']:
            image_files = [source_path]
        elif ext in ['.mp4', '.avi', '.mov', '.mkv']:
            is_video = True
    elif source_path.is_dir():
        for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.webp']:
            image_files.extend(list(source_path.glob(ext)))
        image_files = sorted(image_files)

    out_dir = Path(args.output)
    out_img_dir = out_dir / "annotated_images"
    out_img_dir.mkdir(parents=True, exist_ok=True)
    csv_file = out_dir / "rekapitulasi_pengujian.csv"

    csv_headers = [
        "No", "Nama_File", "Waktu_Inferensi_ms", "ID_Objek", "Kelas_Prediksi",
        "Confidence", "BBox_xyxy", "Sudut_Aktual_deg", "Sudut_Ref_deg",
        "Deviasi_deg", "Koreksi_Servo_deg", "Arah_Servo", "Status_QC", "Aksi_Sistem", "Perintah_UART"
    ]
    hasil_rekap = []
    total_deteksi = 0
    t_start = time.time()

    print(f"  Total gambar yang diuji: {len(image_files)} file\n")
    print(f"  {'No':<4} | {'Nama File':<32} | {'Kelas':<22} | {'Conf':<6} | {'Status QC':<25} | {'Aksi Sistem':<20}")
    print("  " + "-" * 115)

    for img_idx, img_p in enumerate(image_files, 1):
        img_bgr = cv2.imread(str(img_p))
        if img_bgr is None:
            continue

        t0 = time.time()
        results = model.predict(img_bgr, conf=args.conf, iou=args.iou, verbose=False)
        t_inf_ms = (time.time() - t0) * 1000.0

        annotated_img = img_bgr.copy()
        found = False

        for r in results:
            for idx_b, box in enumerate(r.boxes):
                found = True
                total_deteksi += 1
                cls_id = int(box.cls[0].item())
                cls_name = class_names.get(cls_id, str(cls_id))
                conf_val = float(box.conf[0].item())
                xyxy = box.xyxy[0].cpu().numpy().astype(int)

                roi = img_bgr[xyxy[1]:xyxy[3], xyxy[0]:xyxy[2]]
                sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_msg = evaluasi_qc_dan_servo(
                    cls_id, cls_name, roi
                )

                hasil_rekap.append([
                    total_deteksi, img_p.name, f"{t_inf_ms:.1f}", idx_b + 1,
                    cls_name, f"{conf_val:.3f}", f"[{xyxy[0]},{xyxy[1]},{xyxy[2]},{xyxy[3]}]",
                    f"{sudut_act:.1f}", f"{s_ref:.1f}", f"{dev:+.1f}",
                    f"{kor_servo:+.1f}", arah, st_qc, aksi, uart_msg.strip()
                ])

                annotated_img = gambar_anotasi(
                    annotated_img, xyxy, cls_name, conf_val,
                    sudut_act, dev, kor_servo, st_qc, aksi, sudut_ref=s_ref
                )

                file_short = img_p.name[:30] + ".." if len(img_p.name) > 32 else img_p.name
                print(f"  {img_idx:<4} | {file_short:<32} | {cls_name:<22} | {conf_val*100:>5.1f}% | {st_qc:<25} | {aksi:<20}")

        if not found:
            hasil_rekap.append([
                total_deteksi + 1, img_p.name, f"{t_inf_ms:.1f}", 0,
                "Tidak Terdeteksi", "0.000", "[]", "0.0", "0.0",
                "0.0", "0.0", "-", "NO_DETECTION", "ABAIKAN", "-"
            ])
            file_short = img_p.name[:30] + ".." if len(img_p.name) > 32 else img_p.name
            print(f"  {img_idx:<4} | {file_short:<32} | {'(Tidak ada deteksi)':<22} | {'-':>6} | {'NO_DETECTION':<25} | {'ABAIKAN':<20}")

        out_file = out_img_dir / f"result_{img_p.name}"
        cv2.imwrite(str(out_file), annotated_img)

        if args.show:
            cv2.imshow("Hasil Pengujian Offline (SPACE: Next, Q: Keluar)", annotated_img)
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

    with open(csv_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)
        writer.writerows(hasil_rekap)

    durasi = time.time() - t_start
    print("\n" + "=" * 75)
    print(f"  PENGUJIAN SELESAI ({durasi:.2f} detik) | Total Deteksi: {total_deteksi}")
    print(f"  Citra Anotasi : {out_img_dir}")
    print(f"  Laporan CSV   : {csv_file}")
    print("=" * 75 + "\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pengujian Offline Non-Realtime")
    parser.add_argument('--source', type=str, default=str(DEFAULT_TEST_DIR), help='File/folder sumber')
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL), help='Model .pt')
    parser.add_argument('--conf', type=float, default=0.35, help='Confidence threshold')
    parser.add_argument('--iou', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--output', type=str, default=str(DEFAULT_OUTPUT_DIR), help='Folder output')
    parser.add_argument('--show', action='store_true', help='Tampilkan popup OpenCV')
    args = parser.parse_args()
    run_test(args)
