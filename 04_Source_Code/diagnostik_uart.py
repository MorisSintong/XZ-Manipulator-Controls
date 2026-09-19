"""
===============================================================================
  DIAGNOSTIK KONEKSI UART ESP32
  Script ini akan mengecek setiap langkah koneksi dan memberi tahu
  di mana masalahnya.
===============================================================================
"""

import sys
import time
import serial
import serial.tools.list_ports

print("=" * 65)
print("  DIAGNOSTIK KONEKSI UART ESP32")
print("=" * 65)

# ─── STEP 1: Deteksi semua port COM ───
print("\n[STEP 1] Mendeteksi semua port COM yang tersedia...")
ports = list(serial.tools.list_ports.comports())

if not ports:
    print("  ❌ TIDAK ADA PORT COM TERDETEKSI!")
    print("  → Pastikan kabel USB ESP32 sudah dicolokkan")
    print("  → Pastikan driver CH340/CP210x sudah terinstall")
    sys.exit(1)

print(f"  Ditemukan {len(ports)} port:")
for p in ports:
    print(f"    → {p.device:8s} | {p.description} | HWID: {p.hwid}")

# ─── STEP 2: Pilih port ───
target_port = None
keywords = ["cp210", "ch340", "ch341", "usb serial", "ftdi", "espressif", "uart"]
for p in ports:
    desc = (p.description or "").lower()
    hwid = (p.hwid or "").lower()
    for kw in keywords:
        if kw in desc or kw in hwid:
            target_port = p.device
            break
    if target_port:
        break

if not target_port:
    target_port = ports[0].device

print(f"\n[STEP 2] Port yang akan digunakan: {target_port}")

# ─── STEP 3: Coba buka port ───
print(f"\n[STEP 3] Mencoba membuka {target_port}...")

# Metode A: Buka tanpa DTR (agar ESP32 tidak reset)
ser = None
try:
    ser = serial.Serial()
    ser.port = target_port
    ser.baudrate = 115200
    ser.timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    print(f"  ✅ BERHASIL membuka {target_port} (DTR=OFF, RTS=OFF)")
    metode = "tanpa_DTR"
except Exception as e:
    print(f"  ❌ GAGAL membuka {target_port}: {e}")
    print()
    print("  ╔══════════════════════════════════════════════════════════╗")
    print("  ║  PENYEBAB: Port COM sedang dipakai program lain!       ║")
    print("  ║                                                         ║")
    print("  ║  SOLUSI:                                                ║")
    print("  ║  1. TUTUP Serial Monitor di Arduino IDE                 ║")
    print("  ║  2. Tutup program lain yang menggunakan port COM        ║")
    print("  ║  3. Jalankan script ini lagi                            ║")
    print("  ╚══════════════════════════════════════════════════════════╝")
    
    # Coba cek apakah bisa dengan metode normal
    print(f"\n  Mencoba metode alternatif (dengan DTR)...")
    try:
        ser = serial.Serial(target_port, 115200, timeout=1.0)
        print(f"  ✅ BERHASIL dengan metode DTR normal (ESP32 akan ter-reset)")
        metode = "dengan_DTR"
        time.sleep(2.0)  # Tunggu ESP32 boot
    except Exception as e2:
        print(f"  ❌ TETAP GAGAL: {e2}")
        print()
        print("  >>> KESIMPULAN: Port COM TERKUNCI. Tutup Serial Monitor IDE! <<<")
        sys.exit(1)

if ser is None or not ser.is_open:
    print("  ❌ Koneksi serial tidak aktif")
    sys.exit(1)

# ─── STEP 4: Baca data boot ESP32 (jika ada) ───
print(f"\n[STEP 4] Membaca data dari ESP32 (menunggu 2 detik)...")
time.sleep(0.5)

boot_data = ""
t_start = time.time()
while time.time() - t_start < 2.0:
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
        boot_data += chunk
        print(f"  ← ESP32: {chunk.strip()}")
    time.sleep(0.1)

if boot_data:
    print(f"  ✅ ESP32 merespon! ({len(boot_data)} bytes diterima)")
else:
    print(f"  ⚠️ Tidak ada data dari ESP32 (mungkin sudah boot sebelumnya)")

# ─── STEP 5: Kirim PING ───
print(f"\n[STEP 5] Mengirim 'PING' ke ESP32...")
try:
    ser.write(b"PING\n")
    print(f"  → Terkirim: PING")
except Exception as e:
    print(f"  ❌ GAGAL mengirim: {e}")
    ser.close()
    sys.exit(1)

# Tunggu respon
time.sleep(0.5)
respon_ping = ""
t_start = time.time()
while time.time() - t_start < 2.0:
    if ser.in_waiting > 0:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line:
            respon_ping = line
            print(f"  ← ESP32 Respon: {line}")
            break
    time.sleep(0.1)

if "PONG" in respon_ping or "ACK" in respon_ping:
    print(f"  ✅ ESP32 MERESPON PING! Koneksi 2-arah berhasil!")
elif respon_ping:
    print(f"  ⚠️ ESP32 merespon tapi bukan PONG: {respon_ping}")
else:
    print(f"  ❌ ESP32 TIDAK MERESPON PING!")
    print(f"     → Pastikan firmware sudah di-upload ke ESP32")
    print(f"     → Pastikan baudrate 115200 di firmware")

# ─── STEP 6: Kirim paket data koreksi ───
print(f"\n[STEP 6] Mengirim paket koreksi 'K2,S180.0,R180.0'...")
try:
    ser.write(b"K2,S180.0,R180.0\n")
    print(f"  → Terkirim: K2,S180.0,R180.0")
except Exception as e:
    print(f"  ❌ GAGAL mengirim: {e}")
    ser.close()
    sys.exit(1)

# Tunggu ACK
time.sleep(1.0)
respon_data = ""
t_start = time.time()
while time.time() - t_start < 2.0:
    if ser.in_waiting > 0:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line:
            respon_data = line
            print(f"  ← ESP32 ACK: {line}")
            break
    time.sleep(0.1)

if "ACK" in respon_data:
    print(f"  ✅ ESP32 MENERIMA DATA DAN MERESPON!")
elif respon_data:
    print(f"  ⚠️ Respon diterima tapi format tidak sesuai: {respon_data}")
else:
    print(f"  ❌ ESP32 TIDAK MERESPON DATA KOREKSI!")

# ─── STEP 7: Cek log (kirim LOG) ───
print(f"\n[STEP 7] Meminta LOG dari ESP32...")
time.sleep(0.3)
# Buang sisa data di buffer
while ser.in_waiting > 0:
    ser.read(ser.in_waiting)
    time.sleep(0.1)

ser.write(b"LOG\n")
time.sleep(1.0)

log_output = ""
t_start = time.time()
while time.time() - t_start < 3.0:
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
        log_output += chunk
    time.sleep(0.2)

if log_output:
    print("  ← LOG dari ESP32:")
    for line in log_output.strip().split('\n'):
        print(f"     {line.strip()}")
else:
    print("  ❌ Tidak ada output LOG")

# ─── Tutup koneksi ───
ser.dtr = False
ser.rts = False
ser.close()

# ─── RINGKASAN ───
print("\n" + "=" * 65)
print("  RINGKASAN DIAGNOSTIK")
print("=" * 65)
print(f"  Port COM        : {target_port}")
print(f"  Koneksi Serial  : {'✅ OK' if ser else '❌ GAGAL'}")
print(f"  Respon PING     : {'✅ OK' if 'PONG' in respon_ping else '❌ GAGAL'}")
print(f"  Respon Data     : {'✅ OK' if 'ACK' in respon_data else '❌ GAGAL'}")
print(f"  Metode Koneksi  : {metode}")
print("=" * 65)

if 'PONG' not in respon_ping and 'ACK' not in respon_data:
    print()
    print("  MASALAH: ESP32 tidak merespon sama sekali.")
    print("  Kemungkinan penyebab:")
    print("  1. Firmware belum di-upload (upload firmware_esp32.ino)")
    print("  2. Baudrate tidak cocok (harus 115200)")
    print("  3. Port COM salah (coba port lain)")
    print("  4. Kabel USB rusak atau hanya kabel charging")
elif 'PONG' in respon_ping and 'ACK' in respon_data:
    print()
    print("  ✅ SEMUA OK! Koneksi UART berfungsi dengan baik.")
    print("  Anda bisa menjalankan script video test sekarang.")
    print()
    print("  PENTING: Pastikan Serial Monitor IDE TERTUTUP saat")
    print("  menjalankan script Python!")

print()
