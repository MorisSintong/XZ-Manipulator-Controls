"""
Modul Komunikasi Serial UART ke ESP32 / Mikrokontroler
Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
Husein Alhamid (4212301035)

PENTING: Koneksi serial dibuka TANPA sinyal DTR/RTS agar ESP32
         tidak ter-reset saat Python membuka atau menutup port COM.
"""

import time
import serial
import serial.tools.list_ports


def cari_port_esp32() -> str:
    """
    Otomatis mencari port COM ESP32 berdasarkan driver USB-to-UART:
    CP210x, CH340, FTDI, USB Serial, atau Espressif.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    # Prioritas deteksi chip USB serial ESP32
    keywords = ["cp210", "ch340", "ch341", "usb serial", "ftdi", "espressif", "uart"]
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        for kw in keywords:
            if kw in desc or kw in hwid:
                return p.device

    # Jika hanya ada 1 port COM yang tersedia, gunakan port tersebut
    if len(ports) == 1:
        return ports[0].device

    return ports[0].device


class KoneksiUART:
    """
    Wrapper komunikasi UART ke ESP32 dengan fitur:
    - Auto-detection port COM ESP32 (CP210x, CH340, dsb.)
    - Proteksi flooding (cooldown jeda pengiriman data)
    - Auto-reconnect & fallback mode simulasi
    - Penerimaan respon balik (ACK / Telemetri) dari ESP32
    - TANPA reset ESP32 saat buka/tutup port (DTR/RTS disabled)
    """

    def __init__(self, port: str = "AUTO", baudrate: int = 115200, cooldown_sec: float = 0.5):
        self.port_requested = port
        self.port = port
        self.baudrate = baudrate
        self.cooldown_sec = cooldown_sec
        self.waktu_kirim_terakhir = 0.0
        self.pesan_terakhir = ""
        self.ser = None
        self._connect()

    def _connect(self):
        target_port = self.port_requested
        if target_port.upper() == "AUTO":
            detected = cari_port_esp32()
            if detected:
                target_port = detected
            else:
                target_port = "COM3"

        self.port = target_port
        try:
            # ═══════════════════════════════════════════════════════════════
            # PENTING: Buka serial TANPA mengaktifkan DTR/RTS
            # Agar ESP32 TIDAK ter-reset saat Python membuka koneksi.
            # Ini memungkinkan log tetap tersimpan di memori ESP32.
            # ═══════════════════════════════════════════════════════════════
            self.ser = serial.Serial()
            self.ser.port = target_port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = 0.1
            self.ser.dtr = False   # Jangan toggle DTR (penyebab reset ESP32)
            self.ser.rts = False   # Jangan toggle RTS
            self.ser.open()

            # Jeda sebentar agar koneksi stabil
            time.sleep(0.3)

            # Buang data sisa di buffer (jika ada)
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)

            print(f"[UART] Terhubung ke ESP32 pada port: {self.port} @ {self.baudrate} bps")
            print(f"[UART] Mode: DTR/RTS OFF (ESP32 tidak di-reset)")
        except Exception as e:
            print(f"[UART WARNING] ESP32 pada port {self.port} belum terhubung ({e}).")
            print("                Sistem berjalan dalam mode simulasi.")
            self.ser = None

    @property
    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def kirim(self, pesan: str, force: bool = False) -> bool:
        """
        Mengirim string data ke ESP32 (misal: 'K2,S180.0,R180.0\\n').
        Dilengkapi cooldown agar tidak membanjiri buffer serial ESP32.
        """
        sekarang = time.time()
        # Cegah pengiriman berulang data yang sama terlalu cepat
        if not force and (sekarang - self.waktu_kirim_terakhir) < self.cooldown_sec:
            if pesan == self.pesan_terakhir:
                return False

        self.pesan_terakhir = pesan
        self.waktu_kirim_terakhir = sekarang

        if self.is_connected:
            try:
                self.ser.write(pesan.encode('ascii'))
                return True
            except Exception as e:
                print(f"[UART ERROR] Gagal mengirim data ke ESP32: {e}")
                self._connect()
        return False

    def baca_respon(self) -> str:
        """Membaca balasan/ACK dari ESP32 jika ada."""
        if self.is_connected and self.ser.in_waiting > 0:
            try:
                return self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                return ""
        return ""

    def tutup(self):
        """Menutup koneksi serial TANPA mereset ESP32."""
        if self.is_connected:
            try:
                # Pastikan DTR tetap OFF saat menutup agar ESP32 tidak reset
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.close()
                print(f"[UART] Koneksi port {self.port} ditutup (ESP32 tidak di-reset).")
            except Exception:
                pass
