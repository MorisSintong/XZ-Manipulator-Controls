"""Renode Emulation Runner & Controller for STM32F446RE UART Testing.

This module manages the lifecycle of the Renode emulation process and provides
a reliable TCP client interface to communicate with the emulated USART2 peripheral.
It also includes an embedded high-fidelity mock server for headless/CI environments
where Renode may not be installed.
"""

from __future__ import annotations

import argparse
import atexit
import logging
import os
import re
import select
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger("renode_runner")

BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_ELF_PATH = BASE_DIR / "MotorControls" / "MotionFirmware" / "build" / "Debug" / "MotionFirmware.elf"
DEFAULT_RESC_PATH = BASE_DIR / "Emulation" / "renode" / "stm32f446_uart.resc"


def _crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """CRC16-CCITT implementation for mock server validation."""
    crc = init
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


class MockFirmwareServer:
    """In-process mock STM32F446RE firmware server for automated testing and CI.

    Faithfully emulates the STM32 UART receiver ISR ring buffer and protocol
    parser implemented in Core/Src/vision_uart_protocol.c.
    """

    BOOT_BANNER = (
        "\r\n"
        "=================================================================\r\n"
        "   TMC2240 Z-Axis StallGuard4 Sensorless Limit Bouncing Suite    \r\n"
        "   Target: STM32F446RE Nucleo-64 @ 84 MHz                        \r\n"
        "   SPI2: PB13(SCK), PB14(MISO), PB15(MOSI)                      \r\n"
        "   Pins: CS0=PC0, STEP0=PC1, DIR0=PC2, ENN0=PC6                 \r\n"
        "   Telemetry: SEGGER RTT Terminal 0 + USART2 @ 115200 (COM9)    \r\n"
        "=================================================================\r\n"
        "[INFO] Initializing TMC2240 SPI HAL...\r\n"
        "[INFO] Testing SPI connection to TMC2240 (verifying silicon ID)...\r\n"
        "[PASS] TMC2240 Silicon Detected! (Version 0x40 matched)\r\n"
        "[INFO] Configuring TMC2240 motor registers (800mA RMS, StallGuard4)...\r\n"
        "[PASS] TMC2240 StallGuard4 Configured & Ready!\r\n"
        "\r\n"
        "=================================================================\r\n"
        ">>> PRESS BLUE BUTTON (B1 / PC13) TO START Z-AXIS BOUNCING <<<\r\n"
        ">>> (Press Button again anytime during motion to STOP)        <<<\r\n"
        "=================================================================\r\n"
    )

    def __init__(self, host: str = "127.0.0.1", port: int = 12345):
        self.host = host
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self._running = True
        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        logger.info(f"MockFirmwareServer listening on {self.host}:{self.port}")

    def stop(self) -> None:
        self._running = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except Exception:
                pass
            self.client_socket = None

        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("MockFirmwareServer stopped.")

    def _run_server(self) -> None:
        while self._running:
            try:
                self.server_socket.settimeout(0.5)
                client, _ = self.server_socket.accept()
                self.client_socket = client
                self._handle_client(client)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.debug(f"Mock server accept error: {e}")
                break

    def _handle_client(self, client: socket.socket) -> None:
        try:
            client.sendall(self.BOOT_BANNER.encode("latin1"))
        except Exception:
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
            except socket.timeout:
                continue
            except Exception:
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
                    else:
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
        lines = prefix.splitlines()
        for line in lines:
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


class RenodeController:
    """Controls the Renode emulation lifecycle or mock server connection."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 12345,
        resc_path: Optional[str | Path] = None,
        elf_path: Optional[str | Path] = None,
        renode_executable: Optional[str] = None,
        mode: str = "auto",
        startup_timeout: float = 12.0,
    ):
        self.host = host
        self.port = port
        self.mode = mode
        self.startup_timeout = startup_timeout
        self.workspace_root = BASE_DIR

        self.resc_path = Path(resc_path).resolve() if resc_path else DEFAULT_RESC_PATH
        self.elf_path = Path(elf_path).resolve() if elf_path else DEFAULT_ELF_PATH

        self.renode_path = Path(renode_executable or self._find_renode_executable() or "renode")
        self.renode_executable = str(self.renode_path)

        self.renode_process: Optional[subprocess.Popen] = None
        self.mock_server: Optional[MockFirmwareServer] = None
        self.sock: Optional[socket.socket] = None
        self._rx_buffer = bytearray()

    @property
    def proc(self) -> Optional[subprocess.Popen]:
        return self.renode_process

    @staticmethod
    def strip_telnet(data: bytes) -> bytes:
        """Filter RFC 854 Telnet IAC command sequences while preserving 0xFF escapes."""
        out = bytearray()
        i = 0
        n = len(data)
        while i < n:
            if data[i] == 0xFF:
                if i + 1 < n:
                    if data[i + 1] == 0xFF:
                        out.append(0xFF)
                        i += 2
                        continue
                    elif data[i + 1] in (0xFB, 0xFC, 0xFD, 0xFE):  # WILL, WONT, DO, DONT
                        i += 3
                        continue
                    else:
                        i += 2
                        continue
                else:
                    break
            else:
                out.append(data[i])
                i += 1
        return bytes(out)

    @classmethod
    def kill_all_zombies(cls) -> None:
        """Kill any dangling Renode instances to ensure port availability."""
        if sys.platform == "win32":
            subprocess.run(["taskkill", "/F", "/IM", "renode.exe"], capture_output=True)
            subprocess.run(["taskkill", "/F", "/IM", "Renode.exe"], capture_output=True)
        else:
            subprocess.run(["pkill", "-9", "-f", "renode"], capture_output=True)

    def _find_renode_executable(self) -> Optional[str]:
        if "RENODE_PATH" in os.environ and os.path.isfile(os.environ["RENODE_PATH"]):
            return os.environ["RENODE_PATH"]

        found = shutil.which("renode")
        if found:
            return found

        candidates = [
            r"C:\Tools\renode\renode.exe",
            r"C:\Program Files\Renode\bin\renode.exe",
            r"C:\Program Files (x86)\Renode\bin\renode.exe",
            os.path.expanduser("~/renode/renode"),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c
        return None

    def start(self) -> None:
        use_mock = False

        if self.mode == "mock":
            use_mock = True
        elif self.mode == "auto":
            if self._check_socket_open():
                logger.info(f"Existing service detected at {self.host}:{self.port}. Connecting directly.")
                self.connect()
                return

            if not self.renode_path.exists() or not self.resc_path.exists():
                logger.warning("Renode or .resc file not found. Falling back to MockFirmwareServer.")
                use_mock = True
        elif self.mode == "renode":
            if not self.renode_path.exists():
                raise RuntimeError(f"Renode executable not found at {self.renode_path}")
            if not self.resc_path.exists():
                raise FileNotFoundError(f"Renode script does not exist: {self.resc_path}")

        if use_mock:
            logger.info(f"Starting in-process MockFirmwareServer on port {self.port}...")
            self.mock_server = MockFirmwareServer(host=self.host, port=self.port)
            self.mock_server.start()
            time.sleep(0.1)
            self.connect()
            return

        # Start Renode
        logger.info(f"Launching Renode: {self.renode_executable} with {self.resc_path}")
        renode_cmd = [
            str(self.renode_path),
            "--plain",
            "--console",
        ]

        if self.port != 12345:
            renode_cmd.extend(["-e", f"$port={self.port}"])

        if self.elf_path != DEFAULT_ELF_PATH:
            renode_cmd.extend(["-e", f"$bin=@{self.elf_path.as_posix()}"])

        renode_cmd.extend(["-e", f"s @{self.resc_path.as_posix()}"])

        self.renode_process = subprocess.Popen(
            renode_cmd,
            cwd=str(self.workspace_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=False,
        )

        start_time = time.time()
        connected = False
        last_err = None

        while time.time() - start_time < self.startup_timeout:
            if self.renode_process.poll() is not None:
                out = self.renode_process.stdout.read().decode("latin1", errors="replace")
                err = self.renode_process.stderr.read().decode("latin1", errors="replace")
                raise RuntimeError(f"Renode terminated unexpectedly with code {self.renode_process.returncode}:\nSTDOUT: {out}\nSTDERR: {err}")

            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect((self.host, self.port))
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = s
                self._rx_buffer.clear()
                connected = True
                break
            except Exception as e:
                last_err = e
                try:
                    s.close()
                except Exception:
                    pass
                time.sleep(0.3)

        if not connected:
            self.stop()
            raise TimeoutError(f"Could not connect to Renode socket terminal at {self.host}:{self.port} within {self.startup_timeout}s: {last_err}")

        logger.info(f"Renode emulator started and listening on {self.host}:{self.port}")

    def _check_socket_open(self) -> bool:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect((self.host, self.port))
                return True
        except Exception:
            return False

    def connect(self, timeout: float = 5.0) -> None:
        if self.sock is not None:
            return

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((self.host, self.port))
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        self._rx_buffer.clear()
        logger.info(f"Connected to UART socket at {self.host}:{self.port}")

    def connect_socket(self, timeout: float = 3.0) -> socket.socket:
        """Connect and return the underlying socket instance."""
        if self.sock is None:
            self.connect(timeout=timeout)
        return self.sock

    def disconnect(self) -> None:
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._rx_buffer.clear()

    def stop(self) -> None:
        self.disconnect()

        if self.renode_process:
            logger.info("Terminating Renode process...")
            try:
                if self.renode_process.stdout:
                    self.renode_process.stdout.close()
                if self.renode_process.stderr:
                    self.renode_process.stderr.close()
                self.renode_process.terminate()
                self.renode_process.wait(timeout=2.0)
            except Exception:
                try:
                    self.renode_process.kill()
                except Exception:
                    pass
            self.renode_process = None

        if self.mock_server:
            self.mock_server.stop()
            self.mock_server = None

    def is_running(self) -> bool:
        if self.mock_server:
            return True
        if self.renode_process and self.renode_process.poll() is None:
            return True
        return self._check_socket_open()

    def send_raw(self, data: bytes) -> None:
        if not self.sock:
            raise ConnectionError("Socket not connected.")
        self.sock.sendall(data)

    def send_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        self.send_raw(line.encode("latin1"))

    def read_line(self, timeout: float = 3.0) -> str:
        if not self.sock:
            raise ConnectionError("Socket not connected.")

        start_time = time.time()
        while time.time() - start_time < timeout:
            newline_pos = -1
            for i, b in enumerate(self._rx_buffer):
                if b == ord("\n"):
                    newline_pos = i
                    break

            if newline_pos >= 0:
                line_bytes = bytes(self._rx_buffer[: newline_pos + 1])
                del self._rx_buffer[: newline_pos + 1]
                clean = self.strip_telnet(line_bytes)
                return clean.decode("latin1", errors="replace")

            rem = max(0.01, timeout - (time.time() - start_time))
            self.sock.settimeout(rem)
            try:
                chunk = self.sock.recv(1024)
                if not chunk:
                    raise ConnectionResetError("Socket closed by remote target.")
                self._rx_buffer.extend(chunk)
            except socket.timeout:
                pass

        raise TimeoutError(f"Timeout waiting for line after {timeout} seconds (buffered: {bytes(self._rx_buffer)!r})")

    def read_available(
        self,
        sock: Optional[socket.socket] = None,
        timeout: float = 2.0,
        strip_telnet: bool = True,
    ) -> bytes:
        s = sock or self.sock
        if not s:
            raise ConnectionError("No socket connected.")
        accumulated = bytearray()
        start = time.time()
        s.settimeout(0.3)
        while time.time() - start < timeout:
            try:
                chunk = s.recv(4096)
                if not chunk:
                    break
                accumulated.extend(chunk)
            except socket.timeout:
                if accumulated:
                    break
        res = bytes(accumulated)
        if strip_telnet:
            res = self.strip_telnet(res)
        return res

    def drain_banner(self, idle_timeout: float = 0.6, max_timeout: float = 3.0) -> str:
        if not self.sock:
            raise ConnectionError("Socket not connected.")

        accumulated = bytearray()
        start = time.time()
        while time.time() - start < max_timeout:
            self.sock.settimeout(idle_timeout)
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                accumulated.extend(chunk)
            except socket.timeout:
                break

        return self.strip_telnet(bytes(accumulated)).decode("latin1", errors="replace")

    def flush_rx(self) -> None:
        self._rx_buffer.clear()
        if not self.sock:
            return
        self.sock.settimeout(0.05)
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
            except (socket.timeout, BlockingIOError):
                break

    def __enter__(self) -> "RenodeController":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()


def main():
    parser = argparse.ArgumentParser(description="Renode Emulation Controller CLI")
    parser.add_argument("--host", default="127.0.0.1", help="Target TCP host")
    parser.add_argument("--port", type=int, default=12345, help="Target TCP port")
    parser.add_argument("--mode", choices=["auto", "renode", "mock"], default="auto")
    parser.add_argument("--start", action="store_true", help="Start controller and test connection")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    controller = RenodeController(host=args.host, port=args.port, mode=args.mode)
    try:
        controller.start()
        print(f"Successfully connected to controller in {controller.mode} mode!")
        banner = controller.drain_banner()
        print(f"Drained banner ({len(banner)} bytes):\n{banner[:300]}...")
        controller.send_line("PING")
        reply = controller.read_line(timeout=2.0)
        print(f"PING -> {reply.strip()}")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
