# Dokumentasi Testing & Verifikasi Sistem

Direktori ini berisi matriks perencanaan, skenario uji, lembar data pengukuran, dan dokumentasi verifikasi empiris untuk sistem **XZ Manipulator Controls & Vision Sorting**.

---

## 1. Berkas Utama

- **[`Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx`](Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx)**:  
  Workbook pengujian terperinci yang mencakup seluruh skenario validasi fungsional, pengujian presisi, dan uji performa sortasi real-time.

---

## 2. Struktur Matriks Pengujian

Workbook pengujian mencakup lembar kerja (sheets) berikut:

| Kategori Pengujian | Parameter yang Diuji | Target Keberhasilan / Toleransi |
| :--- | :--- | :--- |
| **Uji Komunikasi UART** | Integritas paket 18-byte, CRC16, frame rate, buffer overrun | $0\%$ packet drop rate @ 115200 baud |
| **Uji Sudut Vision** | Principal Component Analysis (PCA) orientasi polaritas kapasitor | Error sudut $\le \pm 2.5^\circ$ pada rentang $[0^\circ, 360^\circ)$ |
| **Uji Tracking Konveyor** | Debounce deteksi objek, antrean FIFO target | 1 kali trigger per kapasitor yang melintas |
| **Uji Driver TMC2240** | Deteksi StallGuard4, homing sensorless, arus termal | Homing konsisten tanpa benturan mekanis keras |
| **Uji Encoder AS5600** | Pembacaan register magnetik 12-bit, noise jitter pada posisi diam | Jitter $\le 2\ \text{LSB}$ |
| **Uji Kinematika Gantry** | Repeatability posisi sumbu $X$ dan $Z$, akurasi translasi | Error posisi $\le \pm 0.2\ \text{mm}$ |
| **Uji End-Effector Vakum** | Waktu pembentukan vakum, holding force, waktu rilis | Waktu respons hisap $< 150\ \text{ms}$ |
| **Uji Sortasi End-to-End** | Siklus pick-inspect-rotate-place pada 100 sampel acak | Akurasi sortasi $\ge 95\%$, cycle time $\le 2.5\ \text{detik}$ |

---

## 3. Metodologi Pengujian & Kriteria Kelulusan

Untuk penjelasan komprehensif mengenai metodologi pengujian berjenjang (V-Model), silakan merujuk ke:  
👉 **[`docs/testing/test_methodology.md`](../docs/testing/test_methodology.md)**

---

## 4. Pelaksanaan Pengujian Otomatis

Sebelum melakukan uji perangkat keras nyata di konveyor, pastikan uji programmatic offline telah lulus:

```powershell
# Verifikasi pipeline visi komputer (Sudut, CRC16, Tracking)
uv run python VisionSystem/tests/verify_vision_system.py

# Verifikasi driver encoder AS5600 (Host CTest)
ctest --test-dir MotorControls/Drivers/AS5600/build --output-on-failure
```
