"""Tests for motion-characterize CLI and replayable report generator."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
from diag.characterize import main
from diag.characterize_report import (
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
