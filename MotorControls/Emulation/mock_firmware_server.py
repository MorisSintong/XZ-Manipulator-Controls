"""In-process STM32H723 mock UART firmware server for offline protocol testing.

Emulates the STM32 UART receiver ISR ring buffer and VisionUart protocol parser
implemented in MotorControls/MotionFirmware/Apps/z_bounce_emulation/Src/vision_uart_protocol.c.
"""

from __future__ import annotations

import argparse
import logging
import re
import socket
import struct
import threading
import time
from typing import Self

logger = logging.getLogger("mock_firmware_server")


def _crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """CRC16-CCITT implementation for mock server validation."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


class MockFirmwareServer:
    """In-process mock STM32H723 firmware server for automated UART protocol testing.

    Emulates the startup banner, legacy ASCII commands (PING, STATUS, K...,S...,R...),
    and 18-byte binary VisionUart frames (0x01 detection, 0x02 heartbeat) with
    CRC16-CCITT validation over a local TCP socket.
    """

    BOOT_BANNER = (
        "\r\n"
        "=================================================================\r\n"
        "   [PROFILE] z_bounce_emulation (Z-Bounce + Vision UART)         \r\n"
        "   TMC2240 Z-Axis StallGuard4 Sensorless Limit Bouncing Suite    \r\n"
        "   Target: STM32H723 SKR Octopus Pro V1.1 @ 400 MHz              \r\n"
        "   SPI1: PA5(SCK), PA6(MISO), PA7(MOSI)                          \r\n"
        "   Pins: CS0=PD11, STEP0=PG0, DIR0=PG1, ENN0=PF15                \r\n"
        "   Telemetry: SEGGER RTT Terminal 0 + USART2 @ 115200 (PD5/PD6)  \r\n"
        "=================================================================\r\n"
        "[INFO] Initializing TMC2240 SPI HAL...\r\n"
        "[INFO] Testing SPI connection to TMC2240 (verifying silicon ID)...\r\n"
        "[PASS] TMC2240 Silicon Detected! (Version 0x40 matched)\r\n"
        "[INFO] Configuring TMC2240 motor registers (800mA RMS, StallGuard4)...\r\n"
        "[PASS] TMC2240 StallGuard4 Configured & Ready!\r\n"
        "\r\n"
        "=================================================================\r\n"
        ">>> PRESS BLUE BUTTON (B1 / PG15) TO START Z-AXIS BOUNCING <<<\r\n"
        ">>> (Press Button again anytime during motion to STOP)        <<<\r\n"
        "=================================================================\r\n"
    )

    def __init__(self, host: str = "127.0.0.1", port: int = 12345):
        self.host = host
        self.port = port
        self.server_socket: socket.socket | None = None
        self.client_socket: socket.socket | None = None
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self._running = True
        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        logger.info("MockFirmwareServer listening on %s:%s", self.host, self.port)

    def stop(self) -> None:
        self._running = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except OSError:
                pass
            self.client_socket = None

        if self.server_socket:
            try:
                self.server_socket.close()
            except OSError:
                pass
            self.server_socket = None

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("MockFirmwareServer stopped.")

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    def _run_server(self) -> None:
        assert self.server_socket is not None
        while self._running:
            try:
                self.server_socket.settimeout(0.5)
                client, _ = self.server_socket.accept()
                self.client_socket = client
                self._handle_client(client)
            except TimeoutError:
                continue
            except OSError as e:
                if self._running:
                    logger.debug("Mock server accept error: %s", e)
                break

    def _handle_client(self, client: socket.socket) -> None:
        try:
            client.sendall(self.BOOT_BANNER.encode("latin1"))
        except OSError:
            return

        rx_buffer = bytearray()
        client.settimeout(0.2)

        while self._running:
            try:
                data = client.recv(1024)
                if not data:
                    break
                rx_buffer.extend(data)
                self._process_rx_stream(client, rx_buffer)
            except TimeoutError:
                continue
            except OSError:
                break

    def _process_rx_stream(self, client: socket.socket, rx_buffer: bytearray) -> None:
        while len(rx_buffer) > 0:
            aa_idx = rx_buffer.find(b"\xAA")
            if aa_idx >= 0:
                if aa_idx > 0:
                    prefix = bytes(rx_buffer[:aa_idx])
                    if b"\n" in prefix or b"\r" in prefix:
                        self._process_ascii_prefix(client, prefix)
                    del rx_buffer[:aa_idx]

                if len(rx_buffer) < 2:
                    break

                if rx_buffer[1] != 0x55:
                    if rx_buffer[1] == 0xAA:
                        del rx_buffer[0]
                        continue
                    client.sendall(b"NACK:FRAME_ERR\r\n")
                    del rx_buffer[:2]
                    continue

                if len(rx_buffer) < 18:
                    break

                frame = bytes(rx_buffer[:18])
                del rx_buffer[:18]
                self._handle_binary_frame(client, frame)
                continue

            newline_idx = -1
            for i, b in enumerate(rx_buffer):
                if b in (ord(b"\n"), ord(b"\r")):
                    newline_idx = i
                    break

            if newline_idx >= 0:
                line_bytes = bytes(rx_buffer[:newline_idx]).strip(b"\r\n")
                del rx_buffer[: newline_idx + 1]
                if line_bytes:
                    self._handle_ascii_line(client, line_bytes.decode("latin1", errors="replace"))
            else:
                if len(rx_buffer) > 128:
                    rx_buffer.clear()
                break

    def _process_ascii_prefix(self, client: socket.socket, prefix: bytes) -> None:
        for line in prefix.splitlines():
            line = line.strip()
            if line:
                self._handle_ascii_line(client, line.decode("latin1", errors="replace"))

    def _handle_binary_frame(self, client: socket.socket, frame: bytes) -> None:
        if len(frame) != 18 or frame[16:18] != b"\x0D\x0A":
            client.sendall(b"NACK:FRAME_ERR\r\n")
            return

        computed_crc = _crc16_ccitt(frame[:14])
        packet_crc = struct.unpack("<H", frame[14:16])[0]

        if computed_crc != packet_crc:
            client.sendall(b"NACK:CRC_ERR\r\n")
            return

        msg_type = frame[2]
        obj_id = struct.unpack("<H", frame[3:5])[0]

        if msg_type == 0x01:
            class_id = frame[5]
            x_val, y_val, angle_val, servo_val = struct.unpack("<h h h h", frame[6:14])
            x_mm = x_val / 10.0
            y_mm = y_val / 10.0
            angle_deg = angle_val / 10.0
            servo_deg = servo_val / 10.0

            reply = (
                f"ACK:OBJ={obj_id},CLS={class_id},X={x_mm:.1f},Y={y_mm:.1f},"
                f"ANG={angle_deg:.1f},SRV={servo_deg:.1f},CRC=OK\r\n"
            )
            client.sendall(reply.encode("latin1"))

        elif msg_type == 0x02:
            reply = f"ACK:HEARTBEAT,SEQ={obj_id}\r\n"
            client.sendall(reply.encode("latin1"))
        else:
            client.sendall(b"NACK:FRAME_ERR\r\n")

    def _handle_ascii_line(self, client: socket.socket, line: str) -> None:
        line = line.strip()
        if line == "PING":
            client.sendall(b"ACK:PONG\r\n")
        elif line == "STATUS":
            client.sendall(b"ACK:STATUS,READY\r\n")
        else:
            m = re.match(r"^K(\d+),S([+-]?\d+(?:\.\d+)?),R([+-]?\d+(?:\.\d+)?)$", line)
            if m:
                cls_id = int(m.group(1))
                corr = float(m.group(3))
                reply = f"ACK:LEGACY,CLS={cls_id},CORR={corr:.1f}\r\n"
                client.sendall(reply.encode("latin1"))
            else:
                client.sendall(b"NACK:FRAME_ERR\r\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="STM32H723 Mock UART Firmware Server")
    parser.add_argument("--host", default="127.0.0.1", help="Bind TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=12345, help="Bind TCP port (default: 12345)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    with MockFirmwareServer(host=args.host, port=args.port):
        print(f"MockFirmwareServer listening on {args.host}:{args.port} (Ctrl+C to stop)")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
