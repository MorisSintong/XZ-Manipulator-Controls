#!/usr/bin/env python3
"""Cross-platform Python Launcher for STM32H723 Mock UART Protocol Test Suite.

Usage:
    python MotorControls/tests/emulation/run_tests.py
    python MotorControls/tests/emulation/run_tests.py --mode mock
    python MotorControls/tests/emulation/run_tests.py --mode connect --host 127.0.0.1 --port 12345
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Launcher for STM32H723 Mock UART Protocol Test Suite"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Target TCP host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=12345, help="Target TCP port (default: 12345)"
    )
    parser.add_argument(
        "--mode",
        choices=["mock", "connect"],
        default="mock",
        help="Execution mode: 'mock' (default) or 'connect' (existing TCP endpoint)",
    )
    parser.add_argument(
        "--json-report",
        default=str(SCRIPT_DIR / "test_results.json"),
        help="JSON output file path",
    )
    args = parser.parse_args()

    test_script = SCRIPT_DIR / "test_stm32_uart_emulation.py"
    cmd = [
        sys.executable,
        str(test_script),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--mode",
        args.mode,
        "--json-report",
        args.json_report,
    ]

    print(f"[RUN] Executing: {' '.join(cmd)}")
    res = subprocess.run(cmd, check=False, cwd=str(REPO_ROOT))
    sys.exit(res.returncode)


if __name__ == "__main__":
    main()
