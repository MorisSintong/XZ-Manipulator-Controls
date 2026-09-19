"""
===============================================================================
  5_test_single_image.py — PENGUJIAN CEPAT 1 GAMBAR SAMPEL
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
===============================================================================
"""

import os
import sys
import io
import argparse
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

import cv2
from ultralytics import YOLO

ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.append(str(ROOT_DIR / "04_Source_Code"))

from utils.angle_calculator import evaluasi_qc_dan_servo, gambar_anotasi
from utils.uart_handler import KoneksiUART

DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"
DEFAULT_SAMPLE_DIR = ROOT_DIR / "05_Hasil_Pengujian" / "sample_images"

def get_default_image():
    if DEFAULT_SAMPLE_DIR.exists():
        for ext in ['*.png', '*.jpg', '*.jpeg', '*.bmp', '*.webp']:
            files = sorted(list(DEFAULT_SAMPLE_DIR.glob(ext)))
            if files:
                return files[0]
    return DEFAULT_SAMPLE_DIR / "ELCOBENAR-P01-A000-Normal-Gerak-S003-VID002__F002__T0.332s.png"

DEFAULT_IMAGE = get_default_image()
OUTPUT_IMAGE = ROOT_DIR / "05_Hasil_Pengujian" / "hasil_single_test.png"

def test_single(args):
    model_path = Path(args.weights)
    image_path = Path(args.image)

    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)
    if not image_path.exists():
        print(f"[ERROR] File gambar tidak ditemukan: {image_path}")
        sys.exit(1)

    print("=" * 65)
    print("  PENGUJIAN DETEKSI & SUDUT TUNGGAL (SINGLE IMAGE)")
    print("=" * 65)
    print(f"  Model  : {model_path.name}")
    print(f"  Gambar : {image_path.name}")
    print("=" * 65)

    model = YOLO(str(model_path))
    class_names = model.names

    img_bgr = cv2.imread(str(image_path))
    results = model.predict(img_bgr, conf=args.conf, iou=0.45, verbose=False)

    annotated_img = img_bgr.copy()
    boxes = results[0].boxes
    print(f"\n[HASIL] Ditemukan {len(boxes)} objek:\n")

    for idx, box in enumerate(boxes, 1):
        cls_id = int(box.cls[0].item())
        cls_name = class_names.get(cls_id, str(cls_id))
        conf_val = float(box.conf[0].item())
        xyxy = box.xyxy[0].cpu().numpy().astype(int)

        roi = img_bgr[xyxy[1]:xyxy[3], xyxy[0]:xyxy[2]]
        sudut_act, s_ref, dev, kor_servo, arah, st_qc, aksi, uart_msg = evaluasi_qc_dan_servo(
            cls_id, cls_name, roi, sudut_manual=args.angle
        )

        print(f"  [{idx}] Kelas       : {cls_name} ({conf_val*100:.1f}%)")
        print(f"      Sudut Aktual: {sudut_act:.1f}° (Input: {args.angle if args.angle is not None else sudut_act}°)")
        print(f"      Status QC   : {st_qc}")
        print(f"      Deviasi     : {dev:+.1f}°")
        print(f"      Koreksi     : {kor_servo:+.1f}° ({arah})")
        print(f"      Aksi Sistem : {aksi}")
        print(f"      Data UART   : {uart_msg.strip()}")
        print("-" * 65)

        annotated_img = gambar_anotasi(
            annotated_img, xyxy, cls_name, conf_val,
            sudut_act, dev, kor_servo, st_qc, aksi, sudut_ref=s_ref
        )

        if args.send_uart:
            uart = KoneksiUART(port=args.uart_port, baudrate=args.baudrate)
            if uart.is_connected:
                uart.kirim(uart_msg, force=True)
                print(f"\n[UART -> ESP32] Berhasil mengirim koreksi: {uart_msg.strip()}")
                time.sleep(0.3)
                ack = uart.baca_respon()
                if ack:
                    print(f"[ESP32 -> UART] Respon Balasan: {ack}")
                uart.tutup()
            else:
                print(f"\n[UART] Gagal terhubung ke ESP32 ({uart.port}). Cek kabel USB atau port COM.")

    out_p = Path(args.output)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_p), annotated_img)
    print(f"\n[✓] Gambar hasil anotasi disimpan di: {out_p}")

    if not args.no_show:
        cv2.imshow("Hasil Deteksi Tunggal (Tekan sembarang tombol untuk keluar)", annotated_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pengujian Single Image + Komunikasi ESP32 UART")
    parser.add_argument('--image', type=str, default=str(DEFAULT_IMAGE), help='Path gambar')
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL), help='Path model .pt')
    parser.add_argument('--conf', type=float, default=0.35, help='Confidence threshold')
    parser.add_argument('--angle', type=float, default=None, help='Simulasi sudut manual (misal: -270, 90, 180)')
    parser.add_argument('--output', type=str, default=str(OUTPUT_IMAGE), help='Path simpan output')
    parser.add_argument('--no-show', action='store_true', help='Jangan tampilkan popup')
    parser.add_argument('--send-uart', action='store_true', help='Kirim hasil koreksi langsung ke ESP32 via UART')
    parser.add_argument('--uart-port', type=str, default="AUTO", help='Port UART ESP32 (AUTO, COM3, COM4, dsb.)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate UART (default: 115200)')
    args = parser.parse_args()
    test_single(args)
