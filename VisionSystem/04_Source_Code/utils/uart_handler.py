"""Binary STM32 transport with one asynchronous serial owner."""

import struct
import time
import uuid
from pathlib import Path

import serial
import serial.tools.list_ports

from .diag_receiver import DiagnosticReceiver
from .vision_command import encode_runtime_detection_packet


def cari_port_esp32() -> str | None:
    """Legacy name retained; prefer an explicit STM32 VCOM port."""
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if len(ports) == 1 else None


class KoneksiUART:
    def __init__(
        self, port="AUTO", baudrate=115200, cooldown_sec=0.5, csv_path=None, config=None
    ):
        self.port_requested = port
        self.port = port
        self.baudrate = baudrate
        self.cooldown_sec = cooldown_sec
        self.waktu_kirim_terakhir = 0.0
        self.pesan_terakhir = b""
        self.csv_path = csv_path or Path("results") / f"vision_diag_{uuid.uuid4()}.csv"
        self.config = config
        self.ser = None
        self.receiver = None
        self._connect()

    def _connect(self):
        self.tutup()
        target = (
            cari_port_esp32()
            if self.port_requested.upper() == "AUTO"
            else self.port_requested
        )
        if not target:
            print("[UART] Specify an unambiguous STM32 port; simulation only.")
            return
        self.port = target
        try:
            self.ser = serial.Serial()
            self.ser.port = target
            self.ser.baudrate = self.baudrate
            self.ser.timeout = 0.05
            self.ser.write_timeout = 0.2
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            self.receiver = DiagnosticReceiver(
                self.ser, self.csv_path, self.config
            ).start()
            print(
                f"[UART] Binary diagnostics connected: {target}; awaiting READY heartbeat."
            )
        except (OSError, ValueError, RuntimeError) as exc:
            self.tutup()
            print(f"[UART] Simulation only: {exc}")

    @property
    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def kirim(self, pesan: bytes, force=False):
        """Return queue admission, not proof of firmware completion or write success."""
        if not isinstance(pesan, bytes):
            raise TypeError("kirim requires the original binary bytes")
        now = time.monotonic()
        if (
            not force
            and now - self.waktu_kirim_terakhir < self.cooldown_sec
            and pesan == self.pesan_terakhir
        ):
            return False

        if self.is_connected and self.receiver and self.receiver.submit(pesan):
            self.waktu_kirim_terakhir = now
            self.pesan_terakhir = pesan
            return True
        return False

    def kirim_deteksi(
        self, obj_id, class_id, x_mm, y_mm, angle_deg, servo_correction_deg, force=False
    ):
        """Build the signed, normalized runtime command; reject bad model outputs."""
        try:
            packet = encode_runtime_detection_packet(
                obj_id, class_id, x_mm, y_mm, angle_deg, servo_correction_deg
            )
        except (ValueError, TypeError, OverflowError, struct.error) as exc:
            if self.receiver:
                self.receiver.reject_admission(exc)
            else:
                print(f"[UART] Admission rejected: {exc}")
            return False
        return self.kirim(packet, force=force)

    @property
    def diagnostic_health(self):
        return (
            self.receiver.health
            if self.receiver
            else {"ready": False, "fault": "disconnected"}
        )

    def tutup(self):
        if self.receiver:
            self.receiver.close()
            self.receiver = None
        if self.ser:
            self.ser.close()
            self.ser = None
