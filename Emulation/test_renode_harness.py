"""
Test Suite for Renode Emulation Harness & USART2 Socket Bridging
================================================================
Verifies headless execution, lifecycle management, socket communication,
and clean process cleanup on Windows.
"""

import socket
import sys
import time
import unittest
from pathlib import Path

# Add workspace and Emulation directories to sys.path
BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR))
sys.path.insert(0, str(BASE_DIR / "Emulation"))

from renode_runner import (
    DEFAULT_ELF_PATH,
    DEFAULT_RESC_PATH,
    RenodeController,
)


class TestRenodeHarness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Clean up any potential zombie processes before tests
        RenodeController.kill_all_zombies()

    @classmethod
    def tearDownClass(cls):
        # Final cleanup pass
        RenodeController.kill_all_zombies()

    def test_strip_telnet_utility(self):
        """Test Telnet IAC sequence filtering."""
        # Standard IAC negotiation sequences
        raw = b"\xff\xfd\x00\xff\xfd\x1f\xff\xfb\x01Hello \xff\xfb\x03World!\xff\xfc\""
        cleaned = RenodeController.strip_telnet(raw)
        self.assertEqual(cleaned, b"Hello World!")

        # Escaped IAC (0xFF 0xFF -> 0xFF)
        raw_escaped = b"\xff\xffData"
        self.assertEqual(RenodeController.strip_telnet(raw_escaped), b"\xffData")

    def test_prerequisites_exist(self):
        """Verify that necessary binaries and resc files exist."""
        ctrl = RenodeController()
        self.assertTrue(
            ctrl.renode_path.exists(),
            f"Renode executable missing at {ctrl.renode_path}",
        )
        self.assertTrue(
            ctrl.resc_path.exists(),
            f"RESC script missing at {ctrl.resc_path}",
        )
        self.assertTrue(
            ctrl.elf_path.exists(),
            f"Firmware ELF missing at {ctrl.elf_path}",
        )

    def test_renode_lifecycle_and_socket_telemetry(self):
        """Test headless launch, socket connection, telemetry reception, and clean shutdown."""
        with RenodeController() as ctrl:
            self.assertTrue(ctrl.is_running(), "Renode process is not running!")
            self.assertIsNotNone(ctrl.proc.pid, "PID should be assigned")

            # Connect socket to sysbus.usart2
            sock = ctrl.connect_socket(timeout=3.0)
            self.assertIsInstance(sock, socket.socket)

            # Receive initial boot banner
            data = ctrl.read_available(sock, timeout=2.0, strip_telnet=True)
            text = data.decode("utf-8", errors="replace")

            # Validate firmware startup telemetry
            self.assertIn("TMC2240", text)
            self.assertIn("STM32F446RE", text)
            self.assertIn("USART2", text)
            self.assertIn("PRESS BLUE BUTTON", text)

        # Confirm termination after context exit
        time.sleep(0.5)
        self.assertFalse(ctrl.is_running(), "Renode process should be terminated after context exit.")

    def test_custom_port_configuration(self):
        """Verify that configurable $port works as expected."""
        alt_port = 12349
        with RenodeController(port=alt_port) as ctrl:
            self.assertTrue(ctrl.is_running())
            sock = ctrl.connect_socket(timeout=3.0)
            self.assertEqual(sock.getpeername()[1], alt_port)
            data = ctrl.read_available(sock, timeout=1.5, strip_telnet=True)
            self.assertGreater(len(data), 0)

        time.sleep(0.5)
        self.assertFalse(ctrl.is_running())


if __name__ == "__main__":
    unittest.main(verbosity=2)
