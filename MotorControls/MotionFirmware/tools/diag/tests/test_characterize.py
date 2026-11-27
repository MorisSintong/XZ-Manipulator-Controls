"""Tests for motion-characterize CLI and replayable report generator."""

from __future__ import annotations

import csv
import json
import shutil
from pathlib import Path

import pytest
from diag.characterize import build_parser, cmd_enc_noise, main
from diag.characterize_report import (
    compute_settle_time_ms,
    generate_report,
    quadratic_fit_accel,
    residual_usteps,
    shortest_angle_error_deg,
    unwrap_counts,
)

FIXTURE_DIR = Path(__file__).parent / "fixtures" / "characterization_v1"


def test_math_primitives():
    # [4095, 0, 1] is a 2-count stationary range, not 4095.
    unwrapped = unwrap_counts([4095, 0, 1])
    assert unwrapped == [0, 1, 2]
    assert max(unwrapped) - min(unwrapped) == 2

    # Reference of 359.9 deg vs 0.0 deg reading gives -0.1 or +0.1 shortest-angle magnitude (0.1 deg), not -359.9.
    assert abs(shortest_angle_error_deg(359.9, 0.0) - 0.1) < 1e-9
    assert abs(shortest_angle_error_deg(0.0, 359.9) - (-0.1)) < 1e-9

    # 3200 commanded edges with 4032 signed encoder counts gives -50 microsteps.
    assert residual_usteps(4032, 3200, 1) == pytest.approx(-50.0)

    # Quadratic fit recovers 2*c2 acceleration cleanly.
    t = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05]
    y = [0.5 * 10000.0 * ti * ti for ti in t]
    accel, rms = quadratic_fit_accel(t, y)
    assert accel == pytest.approx(10000.0, rel=1e-6)
    assert rms is not None and rms < 1e-6


def test_replay_fixture_report(tmp_path: Path):
    run_copy = tmp_path / "run_copy"
    shutil.copytree(FIXTURE_DIR, run_copy)

    rc = main(["report", str(run_copy)])
    assert rc == 0

    summary_path = run_copy / "summary.json"
    report_path = run_copy / "report.md"
    trials_path = run_copy / "trials.csv"
    assert summary_path.exists()
    assert report_path.exists()
    assert trials_path.exists()

    s1 = json.loads(summary_path.read_text(encoding="utf-8"))
    md1 = report_path.read_text(encoding="utf-8")

    # [4095, 0, 1] stationary precision p2p == 2 counts
    assert s1["static_precision"]["x"][0]["p2p_cnt"] == 2

    # 1024 reads in 256000 us -> 4000 reads/s
    assert s1["encoder_rate"]["x_raw_installed"]["highest_repeatable_rate_hz"] == pytest.approx(4000.0)

    # No independent reference -> null absolute_accuracy_deg
    assert s1["reference_accuracy"]["absolute_accuracy_deg"] is None
    assert s1["reference_accuracy"]["reason"] == "no_independent_reference"

    # No theta sensor in xz session -> null theta mechanical, valid theta SPI
    assert s1["motion"]["theta"]["mechanical_status"] == "not_measured"
    assert s1["motion"]["theta"]["reason"] == "no_shaft_feedback"
    assert s1["spi_rate"]["fastest_clean_sck_hz"]["theta"] == 6250000

    # X: 4000 pass + 5000 valid fail -> bracketed
    assert s1["motion"]["x"]["speed_limit_classification"] == "bracketed"
    assert s1["motion"]["x"]["last_pass_speed_steps_s"] == 4000
    assert s1["motion"]["x"]["first_fail_speed_steps_s"] == 5000

    # Z: 2000 pass + 2500 unachieved plateau -> NOT bracketed (cap_reached, invalid trial excluded)
    assert s1["motion"]["z"]["speed_limit_classification"] == "cap_reached"
    assert s1["motion"]["z"]["first_fail_speed_steps_s"] is None
    assert s1["motion"]["z"]["invalid_trials"] == 1

    # Deterministic regeneration produces identical JSON and Markdown
    s2 = generate_report(run_copy)
    md2 = report_path.read_text(encoding="utf-8")
    assert s2 == s1
    assert md2 == md1


def test_setup_unknowns_stay_null(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    out_manifest = tmp_path / "setup.json"
    monkeypatch.setattr("builtins.input", lambda _prompt="": "")
    rc = main(["setup", "--out", str(out_manifest)])
    assert rc == 0
    data = json.loads(out_manifest.read_text(encoding="utf-8"))
    assert data["session_mode"] == "xz"
    assert data["operator"] is None
    assert data["independent_reference_id"] is None
    assert data["axes"]["x"]["run_current_ma"] is None
    assert data["axes"]["z"]["win_neg_steps"] is None


def test_peak_residual_and_settle_time(tmp_path: Path):
    # Direct settle time check including 32-bit timestamp wrap across 0xFFFFFFFF -> 0
    base = 0xFFFFF000
    last_edge = (base + 4000) & 0xFFFFFFFF
    samples = [
        {"t1_us": base, "counts": 0, "residual_usteps": 2},
        {"t1_us": (base + 2000) & 0xFFFFFFFF, "counts": 500, "residual_usteps": -7},
        # Post-move ringing: +10, -5, then settled at 1000 (+/-1) starting at +10000 us (6 ms after last_edge)
        {"t1_us": (base + 6000) & 0xFFFFFFFF, "counts": 1010, "residual_usteps": 8},
        {"t1_us": (base + 8000) & 0xFFFFFFFF, "counts": 995, "residual_usteps": -4},
        {"t1_us": (base + 10000) & 0xFFFFFFFF, "counts": 1001, "residual_usteps": 1},
        {"t1_us": (base + 12000) & 0xFFFFFFFF, "counts": 1000, "residual_usteps": 0},
        {"t1_us": (base + 14000) & 0xFFFFFFFF, "counts": 999, "residual_usteps": -1},
        {"t1_us": (base + 16000) & 0xFFFFFFFF, "counts": 1000, "residual_usteps": 0},
    ]
    assert compute_settle_time_ms(samples, last_edge, settle_band_cnt=2) == pytest.approx(6.0)

    # End-to-end report aggregation: failing trial peak residual must NOT pollute passing axis peak
    run_dir = tmp_path / "run_settle"
    run_dir.mkdir()
    (run_dir / "manifest.json").write_text(
        json.dumps({"capture_id": "cap-settle", "session_mode": "xz"}),
        encoding="utf-8",
    )
    events = [
        {"schema_version": 1, "seq": 1, "trial_id": 1, "kind": "sample", "axis": "x", "t1_us": 10000, "counts": 100, "residual_usteps": -11},
        {"schema_version": 1, "seq": 2, "trial_id": 1, "kind": "sample", "axis": "x", "t1_us": 12000, "counts": 1010, "residual_usteps": 6},
        {"schema_version": 1, "seq": 3, "trial_id": 1, "kind": "sample", "axis": "x", "t1_us": 16000, "counts": 1001, "residual_usteps": 1},
        {"schema_version": 1, "seq": 4, "trial_id": 1, "kind": "sample", "axis": "x", "t1_us": 18000, "counts": 1000, "residual_usteps": 0},
        {"schema_version": 1, "seq": 5, "trial_id": 1, "kind": "sample", "axis": "x", "t1_us": 20000, "counts": 1000, "residual_usteps": 0},
        {
            "schema_version": 1, "seq": 6, "trial_id": 1, "kind": "result", "axis": "x",
            "ok": True, "test": "move", "emitted_edges": 800, "delta_counts": 1024,
            "encoder_sign": 1, "requested_speed_steps_s": 4000, "requested_accel_steps_s2": 10000,
            "max_moving_err_usteps": 12, "settle_spread_cnt": 2, "last_edge_us": 11000,
            "max_lateness_us": 4,
        },
        # Failing trial with huge error: excluded from passing summary
        {
            "schema_version": 1, "seq": 7, "trial_id": 2, "kind": "result", "axis": "x",
            "ok": False, "test": "move", "emitted_edges": 800, "delta_counts": 700,
            "encoder_sign": 1, "requested_speed_steps_s": 8000, "requested_accel_steps_s2": 10000,
            "max_moving_err_usteps": 95, "settle_spread_cnt": 2, "last_edge_us": 30000,
            "max_lateness_us": 4, "reason": "following_error",
        },
    ]
    (run_dir / "events.jsonl").write_text(
        "\n".join(json.dumps(e) for e in events) + "\n",
        encoding="utf-8",
    )

    summary = generate_report(run_dir)
    x_motion = summary["motion"]["x"]
    assert x_motion["peak_abs_residual_usteps"] == pytest.approx(12.0)
    assert x_motion["max_settle_time_ms"] == pytest.approx(5.0)
    assert x_motion["median_settle_time_ms"] == pytest.approx(5.0)

    with (run_dir / "trials.csv").open("r", newline="", encoding="utf-8") as fh:
        csv_rows = list(csv.DictReader(fh))
    assert len(csv_rows) == 2
    assert csv_rows[0]["trial_id"] == "1"
    assert float(csv_rows[0]["peak_abs_residual_usteps"]) == pytest.approx(12.0)
    assert float(csv_rows[0]["settle_time_ms"]) == pytest.approx(5.0)
    assert csv_rows[1]["trial_id"] == "2"
    assert float(csv_rows[1]["peak_abs_residual_usteps"]) == pytest.approx(95.0)


class _FakeEncNoiseSerial:
    """Mirrors mc_core.c: immediate RUN ack, streamed samples, terminal result.

    `running` stays set until the host drains the RX buffer, so a SET or RUN sent
    before the capture's samples are consumed is rejected like the firmware does.
    """

    def __init__(self, reject_run: bool = False) -> None:
        self.rx = bytearray()
        self.sent: list[str] = []
        self.running = False
        self.reject_run = reject_run
        self.sf = 2
        self.n = 64
        self.trial_id = 0

    def _emit(self, rec: dict[str, object]) -> None:
        self.rx.extend((json.dumps(rec) + "\n").encode("utf-8"))

    def write(self, data: bytes) -> int:
        line = data.decode("ascii").strip()
        self.sent.append(line)
        if line in ("KEEP", "STOP"):
            return len(data)
        parts = line.split()
        rid = int(parts[0])
        cmd = parts[1]
        if cmd == "HELLO":
            self._emit({"kind": "hello", "request_id": rid, "profile": "motion_characterization"})
        elif cmd == "SET":
            if self.running:
                self._emit({"kind": "result", "request_id": rid, "ok": False, "reason": "must_be_disarmed"})
            else:
                key, raw_val = parts[2], parts[3]
                if key == "noise_sf":
                    self.sf = int(raw_val)
                elif key == "noise_n":
                    self.n = int(raw_val)
                self._emit({"kind": "result", "request_id": rid, "ok": True})
        elif cmd == "RUN":
            if self.running:
                self._emit({"kind": "result", "request_id": rid, "ok": False, "reason": "busy_or_faulted"})
            elif self.reject_run:
                self._emit({"kind": "result", "request_id": rid, "ok": False, "test": "enc_noise",
                            "reason": "bad_magnet_status"})
            else:
                self._run_enc_noise(rid)
        return len(data)

    def _run_enc_noise(self, rid: int) -> None:
        self.trial_id += 1
        tid = self.trial_id
        self.running = True
        self._emit({"kind": "result", "request_id": rid, "ok": True, "status": "accepted"})
        for i in range(self.n):
            self._emit({"kind": "sample", "request_id": rid, "trial_id": tid, "test": "enc_noise",
                        "idx": i, "raw": 1000, "unwrapped": i})
        self._emit({"kind": "result", "request_id": rid, "trial_id": tid, "ok": True,
                    "test": "enc_noise", "sf": self.sf, "rate_hz": 500, "n": self.n,
                    "errors": 0, "overruns": 0})

    def read(self, n: int) -> bytes:
        out = bytes(self.rx[:n])
        del self.rx[:n]
        if not self.rx:
            self.running = False
        return out

    def close(self) -> None:
        pass


def test_cmd_enc_noise_cli(tmp_path: Path):
    fake = _FakeEncNoiseSerial()
    out_dir = tmp_path / "enc_noise_out"
    args = build_parser().parse_args(
        ["enc-noise", "--port", "COM99", "--axis", "x", "--out", str(out_dir), "--sf", "2,16", "--samples", "64"]
    )
    rc = cmd_enc_noise(args, serial_factory=lambda _port: fake)
    assert rc == 0
    assert (out_dir / "manifest.json").exists()
    assert (out_dir / "events.jsonl").exists()
    assert any("SET noise_sf 2" in s for s in fake.sent)
    assert any("SET noise_sf 16" in s for s in fake.sent)
    assert sum("RUN enc_noise x" in s for s in fake.sent) == 2

    events = [
        json.loads(line)
        for line in (out_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    terminal = [e for e in events if e.get("kind") == "result" and e.get("test") == "enc_noise" and "sf" in e]
    assert sorted(e["sf"] for e in terminal) == [2, 16]
    assert all(e["ok"] is True and e["n"] == 64 and e["errors"] == 0 for e in terminal)
    samples = [e for e in events if e.get("kind") == "sample"]
    assert len(samples) == 2 * 64
    assert sorted(s["idx"] for s in samples if s["trial_id"] == 1) == list(range(64))


def test_cmd_enc_noise_cli_raises_on_reject_and_bad_args(tmp_path: Path):
    def no_serial(_port: str) -> None:
        raise AssertionError("serial must not be opened for invalid arguments")

    bad_argv = (
        ["--rate-hz", "10"],
        ["--rate-hz", "2000"],
        ["--samples", "10"],
        ["--samples", "5000"],
    )
    for i, extra in enumerate(bad_argv):
        out_dir = tmp_path / f"bad_{i}"
        args = build_parser().parse_args(
            ["enc-noise", "--port", "COM99", "--axis", "x", "--out", str(out_dir), *extra]
        )
        with pytest.raises(ValueError):
            cmd_enc_noise(args, serial_factory=no_serial)
        assert not out_dir.exists()

    out_dir = tmp_path / "rejected"
    args = build_parser().parse_args(
        ["enc-noise", "--port", "COM99", "--axis", "x", "--out", str(out_dir), "--sf", "2", "--samples", "64"]
    )
    with pytest.raises(RuntimeError):
        cmd_enc_noise(args, serial_factory=lambda _port: _FakeEncNoiseSerial(reject_run=True))
