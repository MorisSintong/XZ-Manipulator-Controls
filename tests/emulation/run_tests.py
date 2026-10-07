#!/usr/bin/env python3
"""Cross-platform Python Launcher for STM32 UART Emulation Test Suite.

Usage:
    python tests/emulation/run_tests.py
    python tests/emulation/run_tests.py --mode mock
    python tests/emulation/run_tests.py --mode renode --build
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]


def build_firmware_if_needed(force: bool = False) -> bool:
    elf_path = REPO_ROOT / "MotorControls" / "MotionFirmware" / "build" / "Debug" / "MotionFirmware.elf"
    build_dir = REPO_ROOT / "MotorControls" / "MotionFirmware" / "build" / "Debug"
    src_dir = REPO_ROOT / "MotorControls" / "MotionFirmware"

    if force or not elf_path.exists():
        print(f"[BUILD] Rebuilding MotionFirmware -> {elf_path}")
        cmake_bin = shutil.which("cmake")
        ninja_bin = shutil.which("ninja")

        if not cmake_bin or not ninja_bin:
            print("[WARN] cmake or ninja not found; skipping build step.")
            return False

        res = subprocess.run([cmake_bin, "-B", str(build_dir), "-S", str(src_dir)], cwd=str(REPO_ROOT))
        if res.returncode != 0:
            print("[ERROR] CMake configure failed!")
            return False

        res = subprocess.run([ninja_bin, "-C", str(build_dir)], cwd=str(REPO_ROOT))
        if res.returncode != 0:
            print("[ERROR] Ninja build failed!")
            return False

        print("[BUILD] Firmware build complete.")
    return True


def main():
    parser = argparse.ArgumentParser(description="Launcher for STM32 UART Emulation Test Suite")
    parser.add_argument("--host", default="127.0.0.1", help="Target TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=12345, help="Target TCP port (default: 12345)")
    parser.add_argument(
        "--mode",
        choices=["auto", "renode", "mock"],
        default="auto",
        help="Emulation mode (default: auto)",
    )
    parser.add_argument(
        "--json-report",
        default=str(SCRIPT_DIR / "test_results.json"),
        help="JSON output file path",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Force rebuild STM32 firmware before running tests",
    )
    args = parser.parse_args()

    if args.build or args.mode in ("auto", "renode"):
        build_firmware_if_needed(force=args.build)

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
    res = subprocess.run(cmd, cwd=str(REPO_ROOT))
    sys.exit(res.returncode)


if __name__ == "__main__":
    main()
