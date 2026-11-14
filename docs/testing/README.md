# Dokumentasi Testing & Verifikasi Sistem

Direktori ini berisi matriks perencanaan, skenario uji, lembar data pengukuran, dan dokumentasi verifikasi empiris untuk sistem **XZ Manipulator Controls & Vision Sorting**.

---

## 1. Berkas Utama

- **[`Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx`](Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx)**:  
  Workbook pengujian terperinci yang mencakup seluruh skenario validasi fungsional, pengujian presisi, dan uji performa sortasi real-time.
- **[`records/stage1_stallguard_bringup_20261114/README.md`](records/stage1_stallguard_bringup_20261114/README.md)**:  
  Rekaman empiris Stage 1 (`stallguard_cli`: homing repeatability 2x, validasi soft-limit, `mode soft` cycle, plot perbandingan Z `ema_shift`, serta replay Rerun `.rrd`).
- Prosedur verifikasi firmware dan rencana uji bangku tersedia pada [bagian 7 desain motion diagnostics](../architecture/motion_diagnostics_design.md#7-verification-and-bench-plan). Tidak ada berkas `test_methodology.md` terlacak di direktori ini.

---

## 2. Struktur Matriks Pengujian

Workbook pengujian mencakup lembar kerja (sheets) berikut:

Nilai pada tabel berikut adalah target perencanaan, bukan kriteria penerimaan tesis yang sudah diverifikasi. Konfirmasikan toleransi dengan pemilik sistem sebelum menggunakannya untuk menerima perangkat keras.

| Kategori Pengujian | Parameter yang Diuji | Target Perencanaan |
| :--- | :--- | :--- |
| **Uji Komunikasi UART** | Integritas paket 18-byte, CRC16, frame rate, buffer overrun | $0\%$ packet drop rate @ 115200 baud |
| **Uji Sudut Vision** | Principal Component Analysis (PCA) orientitas polaritas kapasitor | Error sudut $\le \pm 2.5^\circ$ pada rentang $[0^\circ, 360^\circ)$ |
| **Uji Tracking Konveyor** | Debounce deteksi objek, antrean FIFO target | 1 kali trigger per kapasitor yang melintas |
| **Uji Driver TMC2240** | Deteksi StallGuard4, homing sensorless, arus termal | Homing konsisten tanpa benturan mekanis keras |
| **Uji Encoder AS5600** | Pembacaan register magnetik 12-bit, noise jitter pada posisi diam | Jitter $\le 2\ \text{LSB}$ |
| **Uji Kinematika Gantry** | Repeatability posisi sumbu $X$ dan $Z$, akurasi translasi | Error posisi $\le \pm 0.2\ \text{mm}$ |
| **Uji End-Effector Vakum** | Waktu pembentukan vakum, holding force, waktu rilis | Waktu respons hisap $< 150\ \text{ms}$ |
| **Uji Sortasi End-to-End** | Siklus pick-inspect-rotate-place pada 100 sampel acak | Akurasi sortasi $\ge 95\%$, cycle time $\le 2.5\ \text{detik}$ |

---

## 3. Referensi Verifikasi

Untuk prosedur verifikasi firmware dan rencana uji perangkat keras, lihat [desain motion diagnostics, bagian 7](../architecture/motion_diagnostics_design.md#7-verification-and-bench-plan). Matriks pengujian kasus nyata dan lembar pengukurannya tetap berada di workbook di atas.

---

## 4. Pelaksanaan Pengujian Otomatis (Offline / Host)

Sebelum melakukan uji perangkat keras nyata di konveyor, jalankan seluruh rangkaian verifikasi terprogram dari direktori *root* repositori:

```powershell
# 1. Host CTest + kompilasi objek ARM Cortex-M7 untuk profil motion_diagnostics (21 pengujian)
.\MotorControls\MotionFirmware\Apps\motion_diagnostics\Tests\run_tests.ps1

# 2. Host CTest untuk profil stallguard_cli (3 suite: motion_profile, units, axis_ctrl; mendukung clang atau gcc)
cmake -S MotorControls\MotionFirmware\Apps\stallguard_cli\tests -B MotorControls\MotionFirmware\Apps\stallguard_cli\tests\build-host -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build MotorControls\MotionFirmware\Apps\stallguard_cli\tests\build-host
ctest --test-dir MotorControls\MotionFirmware\Apps\stallguard_cli\tests\build-host --output-on-failure

# 3. Pustaka bersama motion_diagnostics & CLI host motion-diag (pytest)
uv run --project packages\motion_diagnostics pytest packages\motion_diagnostics\tests
uv run --project MotorControls\MotionFirmware\tools\diag pytest MotorControls\MotionFirmware\tools\diag\tests

# 4. Verifikasi pipeline VisionSystem (PCA sudut kontinu, CRC16, debounce & transport biner)
uv run --project VisionSystem python VisionSystem\tests\verify_vision_system.py
uv run --project VisionSystem pytest VisionSystem\tests

# 5. Verifikasi protokol UART STM32 & Emulasi Mock (11 test cases)
uv run --project VisionSystem python MotorControls\tests\emulation\run_tests.py --mode mock

# 6. Verifikasi driver encoder AS5600 (3 suite Host CTest)
cmake -S MotorControls\Drivers\AS5600 -B MotorControls\Drivers\AS5600\build-host -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
cmake --build MotorControls\Drivers\AS5600\build-host
ctest --test-dir MotorControls\Drivers\AS5600\build-host --output-on-failure

# 7. Verifikasi driver TMC2240 (13 suite Host CTest)
cmake -S MotorControls\Drivers\TMC2240-Driver\Tests -B MotorControls\Drivers\TMC2240-Driver\Tests\build-host -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build MotorControls\Drivers\TMC2240-Driver\Tests\build-host
ctest --test-dir MotorControls\Drivers\TMC2240-Driver\Tests\build-host --output-on-failure
```
