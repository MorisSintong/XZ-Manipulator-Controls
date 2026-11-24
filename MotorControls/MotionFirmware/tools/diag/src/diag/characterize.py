"""Guided host runner CLI for MotionFirmware_Characterization.

Subcommands:
  motion-characterize setup  --out <manifest.json>
  motion-characterize run    --port <COM> --manifest <file> --out <new-dir> --stage passive|motion|reference
  motion-characterize report <run-dir>
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from .characterize_report import generate_report
from .serial_io import open_serial

EXPECTED_PROFILE = "motion_characterization"
EXPECTED_SCHEMA = 1
KEEPALIVE_PERIOD_S = 0.05


def _sha256_file(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _prompt_optional(prompt: str, input_fn: Callable[[str], str] | None = None) -> str | None:
    fn = input_fn if input_fn is not None else input
    val = fn(f"{prompt} [leave blank for null]: ").strip()
    return val if val else None


def _prompt_float(prompt: str, input_fn: Callable[[str], str] | None = None) -> float | None:
    val = _prompt_optional(prompt, input_fn)
    return float(val) if val is not None else None


def _prompt_int(prompt: str, input_fn: Callable[[str], str] | None = None) -> int | None:
    val = _prompt_optional(prompt, input_fn)
    return int(val) if val is not None else None


class CharacterizationClient:
    """Single-owner newline ASCII/NDJSON client over USB-C CDC with background KEEP."""

    def __init__(
        self,
        port: str,
        out_dir: Path,
        serial_factory: Callable[[str], Any] = open_serial,
    ) -> None:
        self.ser = serial_factory(port)
        self.out_dir = out_dir
        self.wire_log = (out_dir / "wire.log").open("ab")
        self.events_log = (out_dir / "events.jsonl").open("a", encoding="utf-8")
        self._lock = threading.Lock()
        self._stop_keep = threading.Event()
        self._keep_enabled = False
        self._req_id = 0
        self._rx_buf = bytearray()
        self._keep_thread = threading.Thread(target=self._keep_loop, daemon=True)
        self._keep_thread.start()

    def _keep_loop(self) -> None:
        while not self._stop_keep.wait(KEEPALIVE_PERIOD_S):
            if self._keep_enabled:
                try:
                    with self._lock:
                        self.ser.write(b"KEEP\n")
                except OSError:
                    break

    def set_keepalive(self, enabled: bool) -> None:
        self._keep_enabled = enabled

    def log_host_event(self, payload: dict[str, Any]) -> None:
        rec = {
            "source": "host",
            "wall_utc": dt.datetime.now(dt.UTC).isoformat(),
            **payload,
        }
        self.events_log.write(json.dumps(rec, sort_keys=True) + "\n")
        self.events_log.flush()

    def send_command(self, cmd: str, timeout_s: float = 5.0) -> list[dict[str, Any]]:
        self._req_id += 1
        rid = self._req_id
        wire = f"{rid} {cmd}\n".encode("ascii")
        self.log_host_event({"kind": "command", "request_id": rid, "cmd": cmd})
        with self._lock:
            written = self.ser.write(wire)
            if written != len(wire):
                raise OSError("short serial write: uncertain execution")

        deadline = time.monotonic() + timeout_s
        collected: list[dict[str, Any]] = []
        while time.monotonic() < deadline:
            chunk = self.ser.read(512)
            if chunk:
                self.wire_log.write(chunk)
                self.wire_log.flush()
                self._rx_buf.extend(chunk)
                while b"\n" in self._rx_buf:
                    idx = self._rx_buf.index(b"\n")
                    raw_line = bytes(self._rx_buf[:idx]).decode("utf-8", errors="replace").strip()
                    del self._rx_buf[: idx + 1]
                    if not raw_line:
                        continue
                    try:
                        obj = json.loads(raw_line)
                    except json.JSONDecodeError:
                        continue
                    self.events_log.write(json.dumps(obj, sort_keys=True) + "\n")
                    self.events_log.flush()
                    collected.append(obj)
                    if obj.get("request_id") == rid and obj.get("kind") in ("hello", "result"):
                        return collected
                    if obj.get("kind") == "event" and obj.get("event") == "fault_latched":
                        return collected
            else:
                time.sleep(0.002)
        raise TimeoutError(f"Timed out waiting for response to request {rid}: {cmd}")

    def send_stop(self) -> None:
        self._keep_enabled = False
        try:
            with self._lock:
                self.ser.write(b"STOP\n")
            self.log_host_event({"kind": "command", "cmd": "STOP"})
        except OSError:
            pass

    def close(self) -> None:
        self._stop_keep.set()
        self._keep_thread.join(timeout=0.5)
        try:
            self.wire_log.close()
        finally:
            try:
                self.events_log.close()
            finally:
                self.ser.close()


def cmd_setup(args: argparse.Namespace, input_fn: Callable[[str], str] | None = None) -> int:
    out_path: Path = args.out
    if out_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing manifest: {out_path}")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    session_mode = _prompt_optional("Session mode (xz | theta_fixture)", input_fn) or "xz"
    if session_mode not in ("xz", "theta_fixture"):
        raise ValueError(f"Unsupported session mode: {session_mode}")

    hex_str = _prompt_optional("Path to flashed MotionFirmware_Characterization.hex", input_fn)
    hex_sha = _sha256_file(Path(hex_str)) if hex_str else None

    manifest: dict[str, Any] = {
        "schema_version": EXPECTED_SCHEMA,
        "capture_id": dt.datetime.now(dt.UTC).strftime("%Y%m%dT%H%M%SZ"),
        "session_mode": session_mode,
        "operator": _prompt_optional("Operator name", input_fn),
        "firmware_hex": hex_str,
        "firmware_sha256": hex_sha,
        "installed_load": _prompt_optional("Installed tooling / payload description", input_fn),
        "vm_supply_nominal_v": _prompt_float("Nominal VM supply (V)", input_fn),
        "cooling": _prompt_optional("Driver cooling condition", input_fn),
        "instrument_clock_evidence": _prompt_optional("Scope / logic analyzer ID (if present)", input_fn),
        "independent_reference_id": _prompt_optional("Independent angle reference ID (if present)", input_fn),
        "independent_reference_uncertainty_deg": _prompt_float("Reference uncertainty (deg)", input_fn),
        "axes": {},
    }

    axes = ("x", "z") if session_mode == "xz" else ("theta",)
    for ax in axes:
        manifest["axes"][ax] = {
            "run_current_ma": _prompt_int(f"[{ax}] Confirmed run current ceiling (mA)", input_fn),
            "hold_pct": _prompt_int(f"[{ax}] Hold current (%)", input_fn),
            "vm_min_mv": _prompt_int(f"[{ax}] VM minimum (mV)", input_fn),
            "vm_max_mv": _prompt_int(f"[{ax}] VM maximum (mV)", input_fn),
            "temp_cutoff_c10": _prompt_int(f"[{ax}] Temp cutoff (0.1 degC, e.g. 1000 = 100C)", input_fn),
            "win_neg_steps": _prompt_int(f"[{ax}] Safe negative clearance (microsteps <= 0)", input_fn),
            "win_pos_steps": _prompt_int(f"[{ax}] Safe positive clearance (microsteps >= 0)", input_fn),
            "min_speed_steps_s": _prompt_int(f"[{ax}] Speed floor (microsteps/s)", input_fn),
            "max_speed_steps_s": _prompt_int(f"[{ax}] Speed ceiling (microsteps/s)", input_fn),
            "max_accel_steps_s2": _prompt_int(f"[{ax}] Accel ceiling (microsteps/s^2)", input_fn),
        }

    out_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Wrote setup manifest: {out_path}")
    return 0


def cmd_run(
    args: argparse.Namespace,
    serial_factory: Callable[[str], Any] = open_serial,
    input_fn: Callable[[str], str] | None = None,
) -> int:
    out_dir: Path = args.out
    if out_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing capture directory: {out_dir}")
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    out_dir.mkdir(parents=True, exist_ok=False)

    run_manifest = dict(manifest)
    run_manifest["stage"] = args.stage
    run_manifest["started_utc"] = dt.datetime.now(dt.UTC).isoformat()
    (out_dir / "manifest.json").write_text(
        json.dumps(run_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    client = CharacterizationClient(args.port, out_dir, serial_factory=serial_factory)
    try:
        # 1. Require exact HELLO profile & schema before sending anything else.
        hello_msgs = client.send_command("HELLO", timeout_s=3.0)
        hello = next((m for m in hello_msgs if m.get("kind") == "hello"), None)
        if (
            hello is None
            or hello.get("profile") != EXPECTED_PROFILE
            or hello.get("schema") != EXPECTED_SCHEMA
        ):
            raise RuntimeError(f"Unexpected firmware handshake: {hello}")

        session_mode = run_manifest.get("session_mode", "xz")
        client.send_command(f"SET session {session_mode}")

        axes = ("x", "z") if session_mode == "xz" else ("theta",)
        if args.stage == "passive":
            for ax in axes:
                client.send_command(f"RUN commission {ax}")
                client.send_command("SET enc_mode raw")
                client.send_command(f"RUN encoder_rate {ax}", timeout_s=15.0)
                client.send_command("SET enc_mode sample")
                client.send_command(f"RUN encoder_rate {ax}", timeout_s=20.0)
                client.send_command(f"RUN static {ax}", timeout_s=15.0)
            if session_mode == "xz":
                client.send_command("SET enc_mode raw")
                client.send_command("RUN encoder_rate both", timeout_s=20.0)
            client.send_command("RUN spi_rate all", timeout_s=30.0)
        elif args.stage == "reference":
            if not run_manifest.get("independent_reference_id"):
                raise RuntimeError("Reference stage requires independent_reference_id in manifest.")
            for ax in axes:
                client.send_command(f"RUN commission {ax}")
                for idx in range(12):
                    fn = input_fn if input_fn is not None else input
                    ref_val = fn(f"[{ax}] Position {idx + 1}/12 reference angle (deg, blank to stop): ").strip()
                    if not ref_val:
                        break
                    res = client.send_command(f"RUN reference {ax}", timeout_s=10.0)
                    client.log_host_event(
                        {
                            "kind": "reference_point",
                            "axis": ax,
                            "reference_deg": float(ref_val),
                            "measured_raw": res[-1].get("first_raw") if res else None,
                            "measured_deg": (
                                float(res[-1]["first_raw"]) * (360.0 / 4096.0)
                                if res and "first_raw" in res[-1]
                                else 0.0
                            ),
                        }
                    )
        elif args.stage == "motion":
            fn = input_fn if input_fn is not None else input
            confirm = fn("Confirm operator present, physical VM cut tested, Z protected [type YES]: ").strip()
            if confirm != "YES":
                raise RuntimeError("Live safety confirmation declined.")
            for ax in axes:
                client.send_command(f"RUN commission {ax}")
                ax_cfg = run_manifest.get("axes", {}).get(ax, {})
                for key, field in (
                    ("run_ma", "run_current_ma"),
                    ("hold_pct", "hold_pct"),
                    ("vm_min_mv", "vm_min_mv"),
                    ("vm_max_mv", "vm_max_mv"),
                    ("temp_cutoff_c10", "temp_cutoff_c10"),
                    ("win_neg", "win_neg_steps"),
                    ("win_pos", "win_pos_steps"),
                    ("min_speed", "min_speed_steps_s"),
                    ("max_speed", "max_speed_steps_s"),
                    ("max_accel", "max_accel_steps_s2"),
                ):
                    if ax_cfg.get(field) is not None:
                        client.send_command(f"SET {key} {ax} {int(ax_cfg[field])}")
                client.send_command(f"SET commissioned {ax} 1")
                client.send_command(f"SET datum_ok {ax} 1")
                client.send_command(f"SET safety_ok {ax} 1")
                client.set_keepalive(True)
                client.send_command(f"ARM {ax}")
                # Gentle 1-full-step sign jog (+16 microsteps)
                client.send_command("POINT 16 200 1000")
                client.send_command(f"RUN jog {ax}", timeout_s=10.0)
                client.send_command("DISARM")
                client.set_keepalive(False)
    except KeyboardInterrupt:
        client.send_stop()
    finally:
        client.close()
        generate_report(out_dir)

    return 0


def cmd_report(args: argparse.Namespace) -> int:
    summary = generate_report(args.run_dir)
    print(f"Generated report in {args.run_dir} (session_mode={summary['session_mode']})")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="motion-characterize",
        description="Motion characterization setup, live bench runner, and replayable report generator.",
    )
    sub = p.add_subparsers(dest="subcommand", required=True)

    p_setup = sub.add_parser("setup", help="Create an operator manifest JSON (unknowns stay null)")
    p_setup.add_argument("--out", required=True, type=Path)

    p_run = sub.add_parser("run", help="Run a live bench stage over USB-C CDC")
    p_run.add_argument("--port", required=True)
    p_run.add_argument("--manifest", required=True, type=Path)
    p_run.add_argument("--out", required=True, type=Path)
    p_run.add_argument("--stage", required=True, choices=("passive", "motion", "reference"))

    p_rep = sub.add_parser("report", help="Replay a capture directory without opening serial")
    p_rep.add_argument("run_dir", type=Path)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.subcommand == "setup":
        return cmd_setup(args)
    if args.subcommand == "run":
        return cmd_run(args)
    if args.subcommand == "report":
        return cmd_report(args)
    raise ValueError(f"Unknown subcommand: {args.subcommand}")


if __name__ == "__main__":
    raise SystemExit(main())
