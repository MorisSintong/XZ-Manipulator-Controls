# 🎓 Proyek Tugas Akhir: Machine Vision Subsystem
### Integrasi Computer Vision dan Sistem Sortasi Konveyor untuk Pengendalian Kualitas (Quality Control) Komponen Kapasitor Berdasarkan Polaritas

* **Penanggung Jawab Machine Vision**: **Husein Alhamid (4212301035)**
* **Mekanikal & Elektrikal**: Moris Sintong HL (4212301036)
* **Dataset & Validasi**: Audi Aulia Ikhsan (4212301058)

---

## 📁 Struktur Direktori Standar Tugas Akhir

Folder ini telah ditata secara modular dan profesional sesuai kaidah dokumentasi proyek rekayasa mekatronika:

```
JOBDESK TA/
│
├── 📁 01_Dokumen_TA/
│   ├── TA - 2026.pdf                     # Dokumen Proposal & Naskah Tugas Akhir
│   └── TA_2026_extracted.txt             # Ekstrak teks naskah untuk referensi
│
├── 📁 02_Dataset/
│   └── Kapasitor_Labeling_1500.v1i.yolo26/
│       ├── data.yaml                     # Konfigurasi kelas & jalur dataset YOLO
│       ├── train/images & labels/        # Data latih model (1.253 citra)
│       ├── valid/images & labels/        # Data validasi (157 citra)
│       └── test/images & labels/         # Data uji independen (156 citra)
│
├── 📁 03_Models/
│   └── best.pt                           # Bobot model YOLO26 terlatih terbaru
│
├── 📁 04_Source_Code/
│   ├── 1_train.py                        # Script training YOLO26 (Anti-Overfitting)
│   ├── 2_evaluate.py                     # Evaluasi kuantitatif (mAP, F1, Confusion Matrix)
│   ├── 3_test_offline.py                 # Pengujian batch dataset test non-realtime
│   ├── 4_detect_realtime.py              # Deteksi kamera live + UART ESP32
│   ├── 5_test_single_image.py            # Pengujian cepat pada 1 gambar sampel
│   ├── 6_test_video.py                   # Pengujian via file video + UART ESP32
│   ├── test_uart_esp32.py                # Pengujian komunikasi serial UART ESP32
│   ├── firmware_esp32/
│   │   └── firmware_esp32.ino            # Firmware ESP32 penerima koreksi servo
│   └── utils/
│       ├── __init__.py
│       ├── angle_calculator.py           # Algoritma perhitungan orientasi & deviasi
│       └── uart_handler.py               # Driver komunikasi serial UART ESP32
│
├── 📁 05_Hasil_Pengujian/
│   ├── sample_images/                    # Gambar contoh input uji
│   ├── evaluation_metrics/               # Visual grafik (Confusion Matrix, PR/F1 Curves)
│   ├── offline_batch_results/            # Hasil anotasi batch 156 citra & CSV
│   │   ├── annotated_images/             # Gambar beranotasi lengkap
│   │   └── rekapitulasi_pengujian.csv    # Rekap data metrik & log pengujian
│   ├── hasil_single_test.png             # Output pengujian single image
│   └── laporan_metrik_evaluasi.txt       # Ringkasan mAP, precision, recall
│
├── 📁 06_Launchers/
│   ├── 1_UJI_OFFLINE.bat                 # Eksekusi batch test (1-klik)
│   ├── 2_UJI_SINGLE_IMAGE.bat            # Eksekusi single image test (1-klik)
│   ├── 3_UJI_REALTIME.bat                # Eksekusi deteksi live kamera (1-klik)
│   ├── 4_EVALUASI_MODEL.bat              # Eksekusi evaluasi metrik TA (1-klik)
│   ├── 5_TEST_UART_ESP32.bat             # Eksekusi pengujian komunikasi UART ESP32
│   └── 6_UJI_VIDEO.bat                   # Eksekusi pengujian via file video (1-klik)
│
├── 🚀 TEST_OFFLINE.bat                   # Shortcut launcher cepat di root
└── 📄 README.md                          # Dokumentasi resmi repositori
```

---

## 📊 Hasil Evaluasi Kuantitatif Model (Dataset Uji 1.500)

Evaluasi kuantitatif pada 156 citra independen (*test split*) dengan checkpoint model terbaru:

| Metrik Evaluasi | Nilai Kinerja | Keterangan |
|---|---|---|
| **mAP@0.5** | **99.03%** | Akurasi lokalisasi & klasifikasi IoU 0.5 |
| **mAP@0.5:0.95** | **90.60%** | Kerapatan bounding box bertingkat tinggi |
| **Precision (Rata-rata)** | **98.80%** | Ketepatan prediksi positif |
| **Recall (Rata-rata)** | **99.42%** | Kemampuan menangkap seluruh objek |
| **F1-Score (Rata-rata)** | **99.11%** | Keseimbangan harmonik Precision & Recall |

### Breakdown Kinerja per Kelas Objek:
| Kelas Objek | Precision ($P$) | Recall ($R$) | mAP@0.5 | mAP@0.5:0.95 |
|---|---|---|---|---|
| **`Elco_Benar`** (Polaritas Benar) | **100.00%** | **100.00%** | **99.50%** | 88.47% |
| **`Elco_Salah`** (Polaritas Terbalik) | **99.31%** | **100.00%** | **99.50%** | 92.45% |
| **`Bukan_Elco`** (Non-Target / Reject) | **97.09%** | **98.26%** | **98.08%** | 90.89% |

---

## ⚙️ Logika Keputusan Quality Control (QC) & Normalisasi Sudut

Sistem menggunakan logika penentuan orientasi berbasis deep learning & pengolahan citra dengan normalisasi sudut $[0^\circ, 360^\circ)$ dan perhitungan koreksi jalur terpendek (*shortest angular path* $[-180^\circ, +180^\circ]$):

$$\theta_{\text{norm}} = (\theta_{\text{input}} \pmod{360} + 360) \pmod{360}$$
$$\text{Deviasi} = (\theta_{\text{norm}} - \theta_{\text{ref}} + 180) \pmod{360} - 180$$
$$\text{Koreksi Servo } (R) = -\text{Deviasi}$$

### Matriks Respons Orientasi & Perintah Servo:
| Kelas Objek | Sudut Input | Sudut Normal | Deviasi | Koreksi Servo ($R$) | Aksi Mekanisme Manipulator | Status QC |
|---|---|---|---|---|---|---|
| **`Elco_Benar`** | $0.0^\circ$ | $0.0^\circ$ | $0.0^\circ$ | **$0.0^\circ$ (DIAM)** | **LANJUT KE JIG (PASS)** | `BENAR` |
| **`Elco_Salah`** | $180.0^\circ$ / $-180^\circ$ | $180.0^\circ$ | $-180.0^\circ$ | **$+180.0^\circ$ (PUTAR 180°)** | **REORIENTASI ($180^\circ$)** | `SALAH` |
| **`Elco_Salah` (Miring Kanan)** | $\mathbf{-270.0^\circ}$ / $+90^\circ$ | $\mathbf{90.0^\circ}$ | $+90.0^\circ$ | **$-90.0^\circ$ (PUTAR CCW 90°)** | **REORIENTASI (CCW $90^\circ$)** | `SALAH` |
| **`Elco_Salah` (Miring Kiri)** | $-90.0^\circ$ / $+270^\circ$ | $270.0^\circ$ | $-90.0^\circ$ | **$+90.0^\circ$ (PUTAR CW 90°)** | **REORIENTASI (CW $90^\circ$)** | `SALAH` |
| **`Bukan_Elco`** | Sembarang | $0.0^\circ$ | $0.0^\circ$ | **$0.0^\circ$ (DIAM)** | **ABAIKAN / REJECT** | `REJECT` |

---

## 📡 Protokol Komunikasi UART ke STM32F446RE

Format pengiriman string data hasil inferensi ke mikrokontroler (Baudrate: **115200 bps**, 8-N-1):

$$\text{Format: } \texttt{K[ID\_Kelas],S[Sudut\_Aktual],R[Koreksi\_Servo]}\backslash\texttt{n}$$

* **Contoh Kapasitor Benar ($0^\circ$)**: $\rightarrow$ `K1,S0.0,R0.0\n`
* **Contoh Kapasitor Salah Terbalik ($180^\circ$)**: $\rightarrow$ `K2,S180.0,R180.0\n`
* **Contoh Kapasitor Miring $-270^\circ$ ($+90^\circ$)**: $\rightarrow$ `K2,S90.0,R-90.0\n`
* **Contoh Kapasitor Miring $-90^\circ$ ($+270^\circ$)**: $\rightarrow$ `K2,S270.0,R90.0\n`
* **Contoh Bukan Elco / Reject**: $\rightarrow$ `K0,S0.0,R0.0\n`

---

## 🚀 Panduan Menjalankan Program

### 1. Pengujian Batch Dataset Test (Non-Realtime)
Jalankan file [`TEST_OFFLINE.bat`](file:///d:/KUMPULAN%20TUGAS%20MEKATRONIKA%20HUSEIN/JOBDESK%20TA/TEST_OFFLINE.bat) atau lewat terminal:
```powershell
python 04_Source_Code\3_test_offline.py
```

### 2. Pengujian Cepat 1 Gambar
```powershell
python 04_Source_Code\5_test_single_image.py --image "05_Hasil_Pengujian\sample_images\ELCOBENAR-P01-A000-Normal-Gerak-S003-VID002__F002__T0.332s.png"
```

### 3. Deteksi Real-Time Kamera + UART
```powershell
python 04_Source_Code\4_detect_realtime.py --source 0 --uart-port COM3
```

### 4. Evaluasi Metrik Tugas Akhir (mAP, F1, Recall, Precision)
```powershell
python 04_Source_Code\2_evaluate.py --split test
```

### 5. Pengujian via File Video + UART ESP32
```powershell
# Dengan ESP32 terhubung (otomatis deteksi port)
python 04_Source_Code\6_test_video.py --video "path\ke\video.mp4" --uart-port AUTO

# Dengan ESP32 pada port manual
python 04_Source_Code\6_test_video.py --video "path\ke\video.mp4" --uart-port COM3

# Tanpa ESP32 (mode offline)
python 04_Source_Code\6_test_video.py --video "path\ke\video.mp4" --no-uart

# Simpan video output beranotasi + kecepatan 0.5x
python 04_Source_Code\6_test_video.py --video "path\ke\video.mp4" --save-video --speed 0.5
```

**Kontrol Playback**: `[SPASI]` Pause/Resume | `[Q]` Keluar | `[S]` Screenshot

### 6. Pengujian Komunikasi UART ESP32
```powershell
python 04_Source_Code\test_uart_esp32.py --port AUTO --angle 180.0
```

---

## 📡 Cara Melihat Output di Serial Monitor Arduino IDE (ESP32)

1. **Upload firmware** `firmware_esp32.ino` ke ESP32 via Arduino IDE
2. **Tutup Serial Monitor** di Arduino IDE (agar port COM tidak dikunci)
3. **Jalankan script Python** (misal: `6_test_video.py` atau `4_detect_realtime.py`)
4. Script Python akan mengirim data UART ke ESP32 dan menampilkan respon ESP32 di terminal
5. **Setelah selesai**, buka kembali Serial Monitor (baudrate: **115200**) untuk melihat log terakhir ESP32

> **Catatan**: Port COM hanya bisa diakses oleh 1 program. Jika Serial Monitor IDE terbuka, script Python tidak bisa mengirim data ke ESP32. Gunakan terminal Python sebagai pengganti Serial Monitor saat menjalankan script.
