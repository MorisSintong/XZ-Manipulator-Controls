"""
===============================================================================
  test_uart_esp32.py — PROGRAM PENGUJIAN KOMUNIKASI SERIAL UART KE ESP32
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
===============================================================================
"""

import sys
import time
import argparse
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.resolve()
sys.path.append(str(ROOT_DIR / "04_Source_Code"))

from utils.uart_handler import KoneksiUART, cari_port_esp32

def main():
    parser = argparse.ArgumentParser(description="Test Komunikasi Serial UART ESP32")
    parser.add_argument('--port', type=str, default="AUTO", help='Port COM ESP32 (misal AUTO, COM3, COM4)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate serial (default: 115200)')
    parser.add_argument('--angle', type=float, default=180.0, help='Sudut koreksi yang ingin diuji (0, 90, 180, -90, dsb.)')
    args = parser.parse_args()

    print("=" * 65)
    print("  PENGUJIAN KOMUNIKASI SERIAL UART ESP32")
    print("=" * 65)

    detected = cari_port_esp32()
    print(f"  Deteksi Otomatis Port : {detected if detected else '(Tidak terdeteksi port aktif)'}")
    print(f"  Port yang Digunakan   : {args.port if args.port != 'AUTO' else (detected or 'COM3')}")
    print(f"  Baudrate              : {args.baudrate} bps")
    print("=" * 65)

    uart = KoneksiUART(port=args.port, baudrate=args.baudrate)

    if not uart.is_connected:
        print("\n[!] PERINGATAN: ESP32 belum terhubung ke komputer.")
        print("    1. Pastikan kabel USB ESP32 sudah dicolokkan ke port USB PC.")
        print("    2. Periksa apakah driver USB-UART (CH340 / CP210x) sudah terpasang.")
        print("    3. Buka Device Manager di Windows dan lihat bagian 'Ports (COM & LPT)'.\n")
        return

    # Kirim PING untuk verifikasi firmware
    print("\n[1] Mengirim sinyal uji 'PING' ke ESP32...")
    uart.kirim("PING\n", force=True)
    time.sleep(0.5)
    ack = uart.baca_respon()
    if ack:
        print(f"    [ESP32 Respon]: {ack}")
    else:
        print("    (Belum ada respon teks dari ESP32, melanjutkan pengiriman perintah...)")

    # Format paket simulasi
    koreksi = args.angle
    cls_id = 1 if abs(koreksi) < 1.0 else 2
    sudut_act = abs(koreksi)
    paket = f"K{cls_id},S{sudut_act:.1f},R{koreksi:.1f}\n"

    print(f"\n[2] Mengirim paket perintah koreksi: {paket.strip()}")
    uart.kirim(paket, force=True)

    # Tunggu balasan ACK dari ESP32
    print("    Menunggu konfirmasi servo dari ESP32...")
    t_start = time.time()
    dapat_ack = False
    while time.time() - t_start < 2.5:
        ack = uart.baca_respon()
        if ack:
            print(f"    [ESP32 ACK] -> {ack}")
            dapat_ack = True
            break
        time.sleep(0.1)

    if dapat_ack:
        print("\n[OK] SUKSES! ESP32 berhasil menerima data dan menggerakkan servo.")
    else:
        print("\n[i] Data telah dikirim. Pastikan firmware di ESP32 sudah sesuai format 'K<id>,S<sudut>,R<koreksi>'.")

    uart.tutup()

if __name__ == "__main__":
    main()
