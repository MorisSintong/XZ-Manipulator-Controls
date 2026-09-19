"""
===============================================================================
  2_evaluate.py — PROGRAM EVALUASI KUANTITATIF MODEL
  Mengisi Tabel 3.3, 3.4, 3.6 (mAP, Precision, Recall, F1, Confusion Matrix)
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

from ultralytics import YOLO

ROOT_DIR = Path(__file__).parent.parent.resolve()
DEFAULT_MODEL = ROOT_DIR / "03_Models" / "best.pt"
DATA_YAML = ROOT_DIR / "02_Dataset" / "Kapasitor_Labeling_1500.v1i.yolo26" / "data.yaml"
OUTPUT_DIR = ROOT_DIR / "05_Hasil_Pengujian"

def evaluate(args):
    model_path = Path(args.weights)
    if not model_path.exists():
        print(f"[ERROR] Bobot model tidak ditemukan: {model_path}")
        sys.exit(1)

    print("=" * 70)
    print("  EVALUASI PERFORMA MODEL YOLO26 PADA DATASET UJI")
    print("  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)")
    print("=" * 70)
    print(f"  Model Weights : {model_path}")
    print(f"  Dataset Config: {DATA_YAML}")
    print(f"  Split         : {args.split}")
    print("=" * 70)

    model = YOLO(str(model_path))
    metrics = model.val(
        data=str(DATA_YAML),
        split=args.split,
        imgsz=640,
        conf=0.25,
        iou=0.6,
        plots=True,
        project=str(OUTPUT_DIR),
        name="evaluation_metrics",
        exist_ok=True
    )

    map50 = metrics.box.map50
    map50_95 = metrics.box.map
    prec = metrics.box.mp
    rec = metrics.box.mr
    f1 = 2 * prec * rec / (prec + rec + 1e-9)

    print("\n" + "=" * 70)
    print("  HASIL EVALUASI UNTUK NASKAH TUGAS AKHIR")
    print("=" * 70)
    print(f"  mAP@0.5               : {map50:.4f} ({map50*100:.2f}%)")
    print(f"  mAP@0.5:0.95          : {map50_95:.4f} ({map50_95*100:.2f}%)")
    print(f"  Precision (Rata-rata) : {prec:.4f} ({prec*100:.2f}%)")
    print(f"  Recall (Rata-rata)    : {rec:.4f} ({rec*100:.2f}%)")
    print(f"  F1-Score (Rata-rata)  : {f1:.4f} ({f1*100:.2f}%)")
    print("=" * 70)

    # Breakdown per kelas
    class_names = metrics.names
    print("\n  BREAKDOWN PER KELAS:")
    print(f"  {'Kelas':<25} | {'Precision':<10} | {'Recall':<10} | {'mAP50':<10} | {'mAP50-95':<10}")
    print("  " + "-" * 68)

    per_class_lines = []
    for cls_idx, cls_name in class_names.items():
        if cls_idx < len(metrics.box.p):
            p_c = metrics.box.p[cls_idx]
            r_c = metrics.box.r[cls_idx]
            map50_c = metrics.box.ap50[cls_idx]
            map_c = metrics.box.ap[cls_idx]
            print(f"  {cls_name:<25} | {p_c:<10.4f} | {r_c:<10.4f} | {map50_c:<10.4f} | {map_c:<10.4f}")
            per_class_lines.append(f"{cls_name:<25} | P: {p_c:.4f} | R: {r_c:.4f} | mAP50: {map50_c:.4f} | mAP50-95: {map_c:.4f}")

    # Simpan laporan teks
    laporan_path = OUTPUT_DIR / "laporan_metrik_evaluasi.txt"
    with open(laporan_path, 'w', encoding='utf-8') as f:
        f.write("=================================================================\n")
        f.write("HASIL PENGUJIAN METRIK MACHINE VISION - HUSEIN ALHAMID (4212301035)\n")
        f.write("=================================================================\n")
        f.write(f"Model Checkpoint : {model_path.name}\n")
        f.write(f"Dataset Evaluasi : Kapasitor_Labeling_1500 ({args.split} split)\n")
        f.write(f"mAP@0.5          : {map50:.4f} ({map50*100:.2f}%)\n")
        f.write(f"mAP@0.5:0.95     : {map50_95:.4f} ({map50_95*100:.2f}%)\n")
        f.write(f"Precision (Avg)  : {prec:.4f} ({prec*100:.2f}%)\n")
        f.write(f"Recall (Avg)     : {rec:.4f} ({rec*100:.2f}%)\n")
        f.write(f"F1-Score (Avg)   : {f1:.4f} ({f1*100:.2f}%)\n\n")
        f.write("RINCIAN PER KELAS:\n")
        for line in per_class_lines:
            f.write(f"  - {line}\n")
        f.write("=================================================================\n")

    print(f"\n[✓] Laporan metrik tersimpan di: {laporan_path}")
    print(f"[✓] Grafik visual (Confusion Matrix, PR Curve, F1 Curve) di: {OUTPUT_DIR / 'evaluation_metrics'}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluasi Model YOLO26")
    parser.add_argument('--weights', type=str, default=str(DEFAULT_MODEL), help='Path model .pt')
    parser.add_argument('--split', type=str, default='test', choices=['val', 'test'], help='Dataset split')
    args = parser.parse_args()
    evaluate(args)
