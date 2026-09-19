"""
===============================================================================
  1_train.py — PROGRAM TRAINING MODEL YOLO26 (ANTI-OVERFITTING)
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

import torch
from ultralytics import YOLO

ROOT_DIR = Path(__file__).parent.parent.resolve()
DATA_YAML = ROOT_DIR / "02_Dataset" / "Kapasitor_Labeling_1500.v1i.yolo26" / "data.yaml"
OUTPUT_MODELS_DIR = ROOT_DIR / "03_Models"

def train(args):
    print("=" * 60)
    print("  MEMULAI PELATIHAN MODEL YOLO26 — QC KAPASITOR")
    print("=" * 60)
    print(f"  Dataset Config : {DATA_YAML}")
    print(f"  Epochs         : {args.epochs}")
    print(f"  Batch Size     : {args.batch}")
    print(f"  Image Size     : {args.imgsz}")
    print(f"  Device         : {'CUDA (GPU)' if torch.cuda.is_available() else 'CPU'}")
    print("=" * 60)

    if not DATA_YAML.exists():
        print(f"[ERROR] File konfigurasi dataset tidak ditemukan: {DATA_YAML}")
        sys.exit(1)

    # Inisialisasi model (Pretrained Transfer Learning)
    model = YOLO(args.model)

    # Pelatihan dengan Hyperparameter Anti-Overfitting
    results = model.train(
        data=str(DATA_YAML),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        patience=args.patience,         # Early stopping
        optimizer="AdamW",              # Optimizer stabil
        lr0=0.001,
        weight_decay=0.0005,            # L2 Regularization
        label_smoothing=0.1,            # Mencegah overconfidence
        dropout=0.1,
        mosaic=1.0,                     # Augmentasi mosaik
        mixup=0.1,
        close_mosaic=10,
        project=str(OUTPUT_MODELS_DIR),
        name="training_run",
        exist_ok=True
    )

    print("\n[✓] Training Selesai! Model terbaik disimpan di 03_Models/training_run/weights/best.pt")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Training YOLO26 Anti-Overfitting")
    parser.add_argument('--model', type=str, default='yolo26m.pt', help='Pretrained model base')
    parser.add_argument('--epochs', type=int, default=100, help='Jumlah epoch')
    parser.add_argument('--batch', type=int, default=16 if torch.cuda.is_available() else 4, help='Batch size')
    parser.add_argument('--imgsz', type=int, default=640 if torch.cuda.is_available() else 320, help='Image resolution')
    parser.add_argument('--patience', type=int, default=30, help='Early stopping patience')
    args = parser.parse_args()
    train(args)
