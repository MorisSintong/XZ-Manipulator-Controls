#!/usr/bin/env python3
"""Automated Emulation Test Suite for STM32F446RE UART Protocol.

Verifies the STM32F446RE MCU's ability to receive, process, validate, and respond
to UART messages sent by the Python Vision System across Renode emulation and QEMU.

Coverage:
- Boot banner and firmware initialization verification
- ASCII commands (PING-PONG handshake, STATUS query)
- Binary framed packets (PASS/FAIL detection packets, Heartbeat)
- Fault injection (corrupted CRC16-CCITT)
- Framing noise recovery and resynchronization
- Rapid burst traffic under line load (FIFO ordering & no packet drops)
- Legacy ASCII format backward compatibility
- Full End-to-End integration alignment with VisionSystem uart_protocol.py
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import os
import re
import socket
import struct
import sys
import time
import unittest
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("stm32_uart_test")

# Workspace root resolution
WORKSPACE_ROOT = Path(__file__).resolve().parents[2]

# Attempt to import RenodeController from Emulation.renode_runner
try:
    sys.path.insert(0, str(WORKSPACE_ROOT))
    from Emulation.renode_runner import RenodeController
except ImportError:
    RenodeController = None

# Dynamically import uart_protocol from VisionSystem
def _load_uart_protocol():
    proto_path = (
        WORKSPACE_ROOT
        / "VisionSystem"
        / "04_Source_Code"
        / "utils"
        / "uart_protocol.py"
    )
    if not proto_path.exists():
        raise FileNotFoundError(f"uart_protocol.py not found at: {proto_path}")
    spec = importlib.util.spec_from_file_location("uart_protocol", proto_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    uart_protocol = _load_uart_protocol()
    encode_detection_packet = uart_protocol.encode_detection_packet
    encode_heartbeat_packet = uart_protocol.encode_heartbeat_packet
    decode_detection_packet = uart_protocol.decode_detection_packet
    crc16_ccitt = uart_protocol.crc16_ccitt
except Exception as e:
    logger.error(f"Failed to import uart_protocol: {e}")
    encode_detection_packet = None
    encode_heartbeat_packet = None
    decode_detection_packet = None
    crc16_ccitt = None


@dataclass
class TestCaseResult:
    test_id: str
    name: str
    description: str
    status: str  # PASS, FAIL, SKIP, ERROR
    duration_ms: float
    details: str
    error: Optional[str] = None


class TestRunReport:
    def __init__(self, target_host: str, target_port: int, mode: str):
        self.target_host = target_host
        self.target_port = target_port
        self.mode = mode
        self.start_time = datetime.now(timezone.utc)
        self.results: List[TestCaseResult] = []

    def add_result(self, res: TestCaseResult) -> None:
        self.results.append(res)

    def print_terminal_summary(self) -> None:
        total = len(self.results)
        passed = sum(1 for r in self.results if r.status == "PASS")
        failed = sum(1 for r in self.results if r.status in ("FAIL", "ERROR"))
        skipped = sum(1 for r in self.results if r.status == "SKIP")
        total_time_ms = sum(r.duration_ms for r in self.results)

        # ANSI styling
        C_RESET = "\033[0m"
        C_GREEN = "\033[1;32m"
        C_RED = "\033[1;31m"
        C_CYAN = "\033[1;36m"
        C_YELLOW = "\033[1;33m"
        C_WHITE = "\033[1;37m"

        col_w_id = 9
        col_w_name = 38
        col_w_status = 8
        col_w_time = 10
        col_w_details = 38

        border = "=" * 109
        sub_border = "-" * 109

        print("\n" + C_CYAN + border + C_RESET)
        print(f"{C_WHITE} STM32F446RE UART PROTOCOL EMULATION VERIFICATION REPORT{C_RESET}")
        print(f" Target: {self.target_host}:{self.target_port} | Mode: {self.mode.upper()} | Time: {self.start_time.strftime('%Y-%m-%d %H:%M:%S UTC')}")
        print(C_CYAN + border + C_RESET)
        print(
            f"{'ID':<{col_w_id}} | {'Test Case':<{col_w_name}} | {'Status':<{col_w_status}} | {'Time (ms)':<{col_w_time}} | {'Details':<{col_w_details}}"
        )
        print(sub_border)

        for r in self.results:
            if r.status == "PASS":
                status_str = f"{C_GREEN}{r.status:<{col_w_status}}{C_RESET}"
            elif r.status in ("FAIL", "ERROR"):
                status_str = f"{C_RED}{r.status:<{col_w_status}}{C_RESET}"
            else:
                status_str = f"{C_YELLOW}{r.status:<{col_w_status}}{C_RESET}"

            details_trunc = (r.details[:col_w_details - 3] + "...") if len(r.details) > col_w_details else r.details
            print(
                f"{r.test_id:<{col_w_id}} | {r.name:<{col_w_name}} | {status_str} | {r.duration_ms:>9.2f} | {details_trunc:<{col_w_details}}"
            )

        print(C_CYAN + border + C_RESET)
        summary_color = C_GREEN if failed == 0 else C_RED
        print(
            f" Results: {summary_color}{passed}/{total} Passed{C_RESET} | "
            f"Failed: {failed} | Skipped: {skipped} | Total Time: {total_time_ms:.1f} ms"
        )
        print(C_CYAN + border + C_RESET + "\n")

    def export_json(self, file_path: Path) -> None:
        file_path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "title": "STM32F446RE UART Emulation Test Suite Results",
            "timestamp": self.start_time.isoformat(),
            "target": {
                "host": self.target_host,
                "port": self.target_port,
                "mode": self.mode,
            },
            "summary": {
                "total": len(self.results),
                "passed": sum(1 for r in self.results if r.status == "PASS"),
                "failed": sum(1 for r in self.results if r.status in ("FAIL", "ERROR")),
                "skipped": sum(1 for r in self.results if r.status == "SKIP"),
                "total_duration_ms": sum(r.duration_ms for r in self.results),
            },
            "test_cases": [asdict(r) for r in self.results],
        }
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        logger.info(f"JSON test results written to: {file_path}")


class STM32UartDirectClient:
    """Direct TCP socket client for communicating with STM32 UART terminal."""

    def __init__(self, host: str = "127.0.0.1", port: int = 12345):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self._rx_buffer = bytearray()
        self.boot_banner = ""

    def connect(self, timeout: float = 6.0, retries: int = 15) -> None:
        last_err = None
        for attempt in range(retries):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(timeout)
                s.connect((self.host, self.port))
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = s
                self._rx_buffer.clear()
                # Drain boot banner
                self.boot_banner = self.drain_banner()
                return
            except Exception as e:
                last_err = e
                time.sleep(0.3)
        raise ConnectionError(f"Failed to connect to STM32 UART at {self.host}:{self.port} after {retries} retries: {last_err}")

    def disconnect(self) -> None:
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._rx_buffer.clear()

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
                raw_line = bytes(self._rx_buffer[: newline_pos + 1])
                del self._rx_buffer[: newline_pos + 1]

                # Strip Telnet IAC sequences if present
                clean = bytearray()
                skip = 0
                for j in range(len(raw_line)):
                    if skip > 0:
                        skip -= 1
                        continue
                    if raw_line[j] == 0xFF and j + 2 < len(raw_line):
                        skip = 2
                        continue
                    clean.append(raw_line[j])

                return bytes(clean).decode("latin1", errors="replace")

            rem = max(0.01, timeout - (time.time() - start_time))
            self.sock.settimeout(rem)
            try:
                chunk = self.sock.recv(1024)
                if not chunk:
                    raise ConnectionResetError("Remote socket connection closed.")
                self._rx_buffer.extend(chunk)
            except socket.timeout:
                pass

        raise TimeoutError(f"Timeout waiting for newline after {timeout}s (buffered: {bytes(self._rx_buffer)!r})")

    def drain_banner(self, idle_timeout: float = 0.5, max_timeout: float = 2.5) -> str:
        if not self.sock:
            return ""
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
        return bytes(accumulated).decode("latin1", errors="replace")

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


class TestSTM32UartEmulation(unittest.TestCase):
    """TestSuite verifying STM32F446RE UART protocol parser implementation."""

    controller: Optional[Any] = None
    client: Optional[STM32UartDirectClient] = None
    boot_banner: str = ""
    mode: str = "auto"
    host: str = "127.0.0.1"
    port: int = 12345
    report: Optional[TestRunReport] = None

    @classmethod
    def setUpClass(cls):
        """Initialize communication channel via RenodeController or direct socket."""
        if cls.report is None:
            cls.report = TestRunReport(target_host=cls.host, target_port=cls.port, mode=cls.mode)

        logger.info(f"Setting up test harness (Mode: {cls.mode}, Target: {cls.host}:{cls.port})...")

        if RenodeController is not None and cls.controller is None:
            try:
                cls.controller = RenodeController(
                    host=cls.host,
                    port=cls.port,
                    mode=cls.mode,
                )
                cls.controller.start()
                cls.boot_banner = cls.controller.drain_banner()
            except Exception as e:
                logger.warning(f"RenodeController startup error: {e}. Falling back to direct client.")
                cls.controller = None

        if cls.controller is None:
            cls.client = STM32UartDirectClient(host=cls.host, port=cls.port)
            cls.client.connect()
            cls.boot_banner = cls.client.boot_banner

    @classmethod
    def tearDownClass(cls):
        """Cleanly terminate test harness."""
        if cls.client:
            cls.client.disconnect()
            cls.client = None

        if cls.controller:
            cls.controller.stop()
            cls.controller = None

    def setUp(self):
        """Ensure clean state before each test case."""
        self._flush_comm()

    def _flush_comm(self) -> None:
        if self.controller:
            self.controller.flush_rx()
        elif self.client:
            self.client.flush_rx()

    def _send_raw(self, data: bytes) -> None:
        if self.controller:
            self.controller.send_raw(data)
        elif self.client:
            self.client.send_raw(data)

    def _send_line(self, line: str) -> None:
        if self.controller:
            self.controller.send_line(line)
        elif self.client:
            self.client.send_line(line)

    def _read_line(self, timeout: float = 2.5) -> str:
        if self.controller:
            return self.controller.read_line(timeout=timeout)
        elif self.client:
            return self.client.read_line(timeout=timeout)
        raise ConnectionError("No active communication link.")

    def _record_result(self, test_id: str, name: str, desc: str, status: str, duration_ms: float, details: str, error: Optional[str] = None):
        if self.report:
            self.report.add_result(TestCaseResult(
                test_id=test_id,
                name=name,
                description=desc,
                status=status,
                duration_ms=duration_ms,
                details=details,
                error=error,
            ))

    # --------------------------------------------------------------------------
    # Test 1: Boot banner and firmware initialization verification
    # --------------------------------------------------------------------------
    def test_01_boot_banner(self):
        """Test 1: Verify firmware welcome banner and initialization strings."""
        t0 = time.perf_counter()
        test_id = "TEST-01"
        name = "Boot Banner & Init Verification"
        desc = "Verify STM32 firmware welcome banner and hardware init"
        try:
            banner = self.__class__.boot_banner
            # If banner wasn't captured initially, check if banner lines are present
            self.assertTrue(len(banner) > 0, "Boot banner should not be empty.")

            # Assert key platform / firmware initialization markers
            expected_markers = [
                "TMC2240",
                "STM32F446RE",
                "Nucleo-64",
                "USART2",
                "SPI HAL",
                "Z-Axis",
                "BOUNCING",
                "BUTTON",
            ]
            matched_markers = [m for m in expected_markers if m in banner]
            self.assertTrue(
                len(matched_markers) >= 1,
                f"Banner must contain at least 1 key platform marker. Matched: {matched_markers}. Banner:\n{banner[:300]}",
            )

            dt_ms = (time.perf_counter() - t0) * 1000.0
            details = f"Verified platform markers: {matched_markers} ({len(banner)} bytes banner)"
            self._record_result(test_id, name, desc, "PASS", dt_ms, details)
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 2: ASCII Ping-Pong handshake
    # --------------------------------------------------------------------------
    def test_02_ascii_ping_pong(self):
        """Test 2: Verify ASCII Ping-Pong handshake ('PING\\n' -> 'ACK:PONG')."""
        t0 = time.perf_counter()
        test_id = "TEST-02"
        name = "ASCII Ping-Pong Handshake"
        desc = "Send 'PING\\n' and assert 'ACK:PONG\\r\\n' reply"
        try:
            self._send_line("PING")
            reply = self._read_line(timeout=2.0).strip()
            self.assertEqual(reply, "ACK:PONG", f"Expected 'ACK:PONG', got '{reply}'")

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Received '{reply}' in {dt_ms:.1f}ms")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 3: ASCII Status command
    # --------------------------------------------------------------------------
    def test_03_ascii_status_command(self):
        """Test 3: Verify ASCII Status query ('STATUS\\n' -> 'ACK:STATUS,READY')."""
        t0 = time.perf_counter()
        test_id = "TEST-03"
        name = "ASCII Status Command"
        desc = "Send 'STATUS\\n' and assert 'ACK:STATUS,READY\\r\\n' reply"
        try:
            self._send_line("STATUS")
            reply = self._read_line(timeout=2.0).strip()
            self.assertEqual(reply, "ACK:STATUS,READY", f"Expected 'ACK:STATUS,READY', got '{reply}'")

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Received '{reply}'")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 4: Binary Detection Packet (PASS object)
    # --------------------------------------------------------------------------
    def test_04_binary_detection_pass(self):
        """Test 4: Binary detection packet for PASS object (Class 1, Angle 0°, Servo 0°)."""
        t0 = time.perf_counter()
        test_id = "TEST-04"
        name = "Binary Detection (PASS: Class 1)"
        desc = "Send binary framed detection packet (Obj=101, Cls=1, Ang=0°, Srv=0°)"
        try:
            obj_id = 101
            class_id = 1
            x_mm = 120.0
            y_mm = 250.0
            angle_deg = 0.0
            servo_deg = 0.0

            packet = encode_detection_packet(
                obj_id=obj_id,
                class_id=class_id,
                x_mm=x_mm,
                y_mm=y_mm,
                angle_deg=angle_deg,
                servo_correction_deg=servo_deg,
            )
            self.assertEqual(len(packet), 18, "Packet size must be 18 bytes")

            self._send_raw(packet)
            reply = self._read_line(timeout=2.0).strip()

            expected_pattern = r"^ACK:OBJ=101,CLS=1,X=120\.0,Y=250\.0,ANG=0\.0,SRV=0\.0,CRC=OK$"
            self.assertTrue(
                re.match(expected_pattern, reply),
                f"Reply '{reply}' did not match pattern '{expected_pattern}'",
            )

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Validated: '{reply}'")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 5: Binary Detection Packet (FAIL object)
    # --------------------------------------------------------------------------
    def test_05_binary_detection_fail(self):
        """Test 5: Binary detection packet for FAIL object (Class 2, Angle 180°, Servo 90°)."""
        t0 = time.perf_counter()
        test_id = "TEST-05"
        name = "Binary Detection (FAIL: Class 2)"
        desc = "Send binary framed detection packet (Obj=202, Cls=2, Ang=180°, Srv=90°)"
        try:
            obj_id = 202
            class_id = 2
            x_mm = 175.5
            y_mm = 310.2
            angle_deg = 180.0
            servo_deg = 90.0

            packet = encode_detection_packet(
                obj_id=obj_id,
                class_id=class_id,
                x_mm=x_mm,
                y_mm=y_mm,
                angle_deg=angle_deg,
                servo_correction_deg=servo_deg,
            )
            self.assertEqual(len(packet), 18, "Packet size must be 18 bytes")

            self._send_raw(packet)
            reply = self._read_line(timeout=2.0).strip()

            expected_pattern = r"^ACK:OBJ=202,CLS=2,X=175\.5,Y=310\.2,ANG=180\.0,SRV=90\.0,CRC=OK$"
            self.assertTrue(
                re.match(expected_pattern, reply),
                f"Reply '{reply}' did not match expected pattern: '{expected_pattern}'",
            )

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Validated: '{reply}'")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 6: Binary Heartbeat Packet
    # --------------------------------------------------------------------------
    def test_06_binary_heartbeat(self):
        """Test 6: Verify Binary Heartbeat Packet (Seq=555 -> 'ACK:HEARTBEAT,SEQ=555')."""
        t0 = time.perf_counter()
        test_id = "TEST-06"
        name = "Binary Heartbeat Packet"
        desc = "Send binary heartbeat frame (Type=0x02, Seq=555) and verify ACK"
        try:
            seq = 555
            packet = encode_heartbeat_packet(seq=seq)
            self.assertEqual(len(packet), 18, "Heartbeat packet size must be 18 bytes")

            self._send_raw(packet)
            reply = self._read_line(timeout=2.0).strip()

            expected = f"ACK:HEARTBEAT,SEQ={seq}"
            self.assertEqual(reply, expected, f"Expected '{expected}', got '{reply}'")

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Confirmed: '{reply}'")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 7: Fault injection — corrupted CRC byte
    # --------------------------------------------------------------------------
    def test_07_corrupted_crc_fault_injection(self):
        """Test 7: Fault injection — assert 'NACK:CRC_ERR' upon receiving corrupted CRC16."""
        t0 = time.perf_counter()
        test_id = "TEST-07"
        name = "Fault Injection (Corrupted CRC16)"
        desc = "Invert CRC16 bits in detection packet and assert 'NACK:CRC_ERR'"
        try:
            packet = bytearray(encode_detection_packet(
                obj_id=301,
                class_id=1,
                x_mm=50.0,
                y_mm=50.0,
                angle_deg=45.0,
                servo_correction_deg=10.0,
            ))
            # Corrupt CRC LSB byte at index 14
            packet[14] ^= 0xFF

            self._send_raw(bytes(packet))
            reply = self._read_line(timeout=2.0).strip()

            self.assertEqual(reply, "NACK:CRC_ERR", f"Expected 'NACK:CRC_ERR', got '{reply}'")

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Fault caught: '{reply}'")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 8: Framing noise recovery
    # --------------------------------------------------------------------------
    def test_08_framing_noise_recovery(self):
        """Test 8: Framing noise recovery — leading garbage followed immediately by valid packet."""
        t0 = time.perf_counter()
        test_id = "TEST-08"
        name = "Framing Noise Recovery"
        desc = "Inject leading noise and corrupted preamble bytes, assert clean resync"
        try:
            valid_pkt = encode_heartbeat_packet(seq=888)

            # Test A: False preamble noise (0xAA followed by non-0x55 bytes)
            # STM32 parser emits NACK:FRAME_ERR on false preamble, then immediately resyncs
            noise_stream = b"\xAA\x33\xAA\x77" + valid_pkt
            self._send_raw(noise_stream)

            # Drain responses until we receive the valid heartbeat ACK
            received_replies = []
            ack_found = False
            start_wait = time.time()
            while time.time() - start_wait < 3.0:
                try:
                    line = self._read_line(timeout=1.0).strip()
                    received_replies.append(line)
                    if line == "ACK:HEARTBEAT,SEQ=888":
                        ack_found = True
                        break
                except TimeoutError:
                    break

            self.assertTrue(
                ack_found,
                f"Parser failed to resynchronize after preamble noise. Received: {received_replies}",
            )

            # Test B: Arbitrary line noise before valid detection packet
            det_pkt = encode_detection_packet(401, 1, 99.0, 88.0, 0.0, 0.0)
            noise_b = b"\x00\xFF\x12\x34\x56" + det_pkt
            self._send_raw(noise_b)

            reply_b = self._read_line(timeout=2.0).strip()
            self.assertTrue(
                "ACK:OBJ=401" in reply_b,
                f"Expected clean ACK for obj 401 after arbitrary noise, got '{reply_b}'",
            )

            dt_ms = (time.perf_counter() - t0) * 1000.0
            details = f"Clean recovery verified across false preambles & raw noise: {received_replies[-1]}"
            self._record_result(test_id, name, desc, "PASS", dt_ms, details)
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 9: Rapid burst test (10 packets back-to-back)
    # --------------------------------------------------------------------------
    def test_09_rapid_burst_traffic(self):
        """Test 9: Rapid burst test — send 10 packets rapidly back-to-back, verify all are ACKed in order."""
        t0 = time.perf_counter()
        test_id = "TEST-09"
        name = "Rapid Burst Stress Test (10 Frames)"
        desc = "Stream 10 binary detection packets without inter-frame delay; assert in-order FIFO ACKs"
        burst_count = 10
        try:
            burst_packets = []
            expected_ids = []

            for i in range(1, burst_count + 1):
                oid = 700 + i
                expected_ids.append(oid)
                cls_id = 1 if (i % 2 == 1) else 2
                x_val = float(i * 10)
                y_val = float(i * 15)
                ang_val = float((i * 30) % 360)
                srv_val = float((i * 15) % 180)
                pkt = encode_detection_packet(oid, cls_id, x_val, y_val, ang_val, srv_val)
                burst_packets.append(pkt)

            # Stream all 10 packets in a single socket write (180 bytes)
            all_bytes = b"".join(burst_packets)
            self._send_raw(all_bytes)

            # Read all 10 ACK replies
            received_acks = []
            for i in range(burst_count):
                line = self._read_line(timeout=2.5).strip()
                received_acks.append(line)

            self.assertEqual(
                len(received_acks),
                burst_count,
                f"Expected {burst_count} ACKs, received {len(received_acks)}",
            )

            # Verify each ACK matches in sequence
            for idx, (expected_oid, ack_line) in enumerate(zip(expected_ids, received_acks), start=1):
                self.assertTrue(
                    f"ACK:OBJ={expected_oid}" in ack_line,
                    f"Frame {idx} mismatch: expected OBJ={expected_oid}, got '{ack_line}'",
                )
                self.assertTrue(
                    "CRC=OK" in ack_line,
                    f"Frame {idx} missing CRC=OK: '{ack_line}'",
                )

            dt_ms = (time.perf_counter() - t0) * 1000.0
            throughput_fps = (burst_count / (dt_ms / 1000.0))
            details = f"All {burst_count} frames ACKed in exact FIFO sequence ({throughput_fps:.1f} fps)"
            self._record_result(test_id, name, desc, "PASS", dt_ms, details)
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 10: Legacy format test
    # --------------------------------------------------------------------------
    def test_10_legacy_ascii_format(self):
        """Test 10: Legacy format test — 'K2,S180.0,R90.0\\n' -> 'ACK:LEGACY,CLS=2,CORR=90.0'."""
        t0 = time.perf_counter()
        test_id = "TEST-10"
        name = "Legacy ASCII Format Compatibility"
        desc = "Send legacy format 'K%d,S%f,R%f\\n' and verify backwards-compatible ACK"
        try:
            legacy_cmd = "K2,S180.0,R90.0\n"
            self._send_line(legacy_cmd)

            reply = self._read_line(timeout=2.0).strip()
            expected = "ACK:LEGACY,CLS=2,CORR=90.0"
            self.assertEqual(reply, expected, f"Expected '{expected}', got '{reply}'")

            # Second legacy case with zero angles
            self._send_line("K1,S0.0,R0.0\n")
            reply2 = self._read_line(timeout=2.0).strip()
            expected2 = "ACK:LEGACY,CLS=1,CORR=0.0"
            self.assertEqual(reply2, expected2, f"Expected '{expected2}', got '{reply2}'")

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "PASS", dt_ms, f"Validated legacy replies: '{reply}', '{reply2}'")
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise

    # --------------------------------------------------------------------------
    # Test 11: End-to-End integration test using uart_protocol.py
    # --------------------------------------------------------------------------
    def test_11_e2e_uart_protocol_alignment(self):
        """Test 11: End-to-End protocol alignment directly importing VisionSystem uart_protocol.py."""
        t0 = time.perf_counter()
        test_id = "TEST-11"
        name = "End-to-End Protocol Alignment"
        desc = "Verify full round-trip symmetry: Python encode -> decode -> STM32 processing -> reply"
        try:
            self.assertIsNotNone(encode_detection_packet, "VisionSystem uart_protocol could not be loaded")
            self.assertIsNotNone(decode_detection_packet, "decode_detection_packet function missing")
            self.assertIsNotNone(crc16_ccitt, "crc16_ccitt function missing")

            # 1. Parameter set
            obj_id = 999
            class_id = 1
            x_mm = 142.3
            y_mm = 217.8
            angle_deg = 270.5
            servo_deg = -45.0  # Normalized signed correction

            # 2. Python encode
            raw_packet = encode_detection_packet(
                obj_id=obj_id,
                class_id=class_id,
                x_mm=x_mm,
                y_mm=y_mm,
                angle_deg=angle_deg,
                servo_correction_deg=servo_deg,
            )
            self.assertEqual(len(raw_packet), 18, "Encoded packet length must be exactly 18 bytes")

            # 3. Verify Python-side decoder can unpack and validate its own packet
            decoded = decode_detection_packet(raw_packet)
            self.assertIsNotNone(decoded, "Python decode_detection_packet returned None on freshly encoded packet")
            self.assertEqual(decoded["obj_id"], obj_id)
            self.assertEqual(decoded["class_id"], class_id)
            self.assertAlmostEqual(decoded["x_mm"], x_mm, places=1)
            self.assertAlmostEqual(decoded["y_mm"], y_mm, places=1)
            self.assertAlmostEqual(decoded["angle_deg"], angle_deg, places=1)
            self.assertAlmostEqual(decoded["servo_correction_deg"], servo_deg, places=1)

            # 4. Verify CRC16 algorithm matches STM32 CCITT polynomial (0x1021, init 0xFFFF)
            crc_calc = crc16_ccitt(raw_packet[:14])
            crc_in_pkt = struct.unpack("<H", raw_packet[14:16])[0]
            self.assertEqual(crc_calc, crc_in_pkt, "Calculated CRC must match packet bytes")

            # 5. Transmit to STM32 and verify reply matches
            self._send_raw(raw_packet)
            reply = self._read_line(timeout=2.0).strip()

            expected_ack = f"ACK:OBJ={obj_id},CLS={class_id},X={x_mm:.1f},Y={y_mm:.1f},ANG={angle_deg:.1f},SRV={servo_deg:.1f},CRC=OK"
            self.assertEqual(reply, expected_ack, f"STM32 ACK mismatch: expected '{expected_ack}', got '{reply}'")

            dt_ms = (time.perf_counter() - t0) * 1000.0
            details = f"Complete protocol symmetry verified across Python & STM32: OBJ={obj_id}"
            self._record_result(test_id, name, desc, "PASS", dt_ms, details)
        except Exception as e:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_result(test_id, name, desc, "FAIL", dt_ms, str(e), str(e))
            raise


def run_tests_standalone(
    host: str = "127.0.0.1",
    port: int = 12345,
    mode: str = "auto",
    json_path: Optional[Path] = None,
) -> int:
    """Run test suite and format report. Returns exit code 0 for pass, 1 for fail."""
    report = TestRunReport(target_host=host, target_port=port, mode=mode)
    TestSTM32UartEmulation.host = host
    TestSTM32UartEmulation.port = port
    TestSTM32UartEmulation.mode = mode
    TestSTM32UartEmulation.report = report

    suite = unittest.TestLoader().loadTestsFromTestCase(TestSTM32UartEmulation)
    runner = unittest.TextTestRunner(verbosity=0)
    test_result = runner.run(suite)

    # Print terminal report
    report.print_terminal_summary()

    # Export JSON report
    if json_path is None:
        json_path = WORKSPACE_ROOT / "tests" / "emulation" / "test_results.json"
    report.export_json(json_path)

    return 0 if test_result.wasSuccessful() else 1


def main():
    parser = argparse.ArgumentParser(description="STM32F446RE UART Emulation Automated Test Suite")
    parser.add_argument("--host", default="127.0.0.1", help="Target TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=12345, help="Target TCP port (default: 12345)")
    parser.add_argument(
        "--mode",
        choices=["auto", "renode", "mock"],
        default="auto",
        help="Emulation execution mode: 'renode' (launch Renode), 'mock' (in-process mock server), 'auto' (detect/start)",
    )
    parser.add_argument(
        "--json-report",
        type=str,
        default=None,
        help="Path to output JSON test report (default: tests/emulation/test_results.json)",
    )
    args = parser.parse_args()

    json_path = Path(args.json_report).resolve() if args.json_report else None
    exit_code = run_tests_standalone(
        host=args.host,
        port=args.port,
        mode=args.mode,
        json_path=json_path,
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
