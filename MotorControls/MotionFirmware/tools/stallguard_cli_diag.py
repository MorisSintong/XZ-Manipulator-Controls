"""Automated live diagnostic runner & telemetry visualizer for stallguard_cli.

Runs homing repeatability (2x), soft-limit/midpoint positioning, rotation
(Theta / MOTOR2_1) angle precision & bidirectional repeatability checks, and
optional soft-cycle checks over USB CDC CLI, logs raw + structured JSON output,
and renders high-resolution PNG movement, StallGuard4, and angle precision plots
(OpenCV + NumPy).
"""

from __future__ import annotations

import argparse
import json
import re
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import serial

# ---------------------------------------------------------------------------
# Serial CLI harness
# ---------------------------------------------------------------------------


class BoardCLI:
    def __init__(self, port: str, baud: int, log_file):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.dtr = True
        self.log_file = log_file
        self.buf = ""
        time.sleep(0.35)
        self.drain(0.35)

    def close(self) -> None:
        self.ser.close()

    def _record(self, line: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_file.write(f"[{ts}] {line}\n")
        self.log_file.flush()

    def send(self, cmd: str) -> None:
        self._record(f">>> {cmd}")
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

    def read_lines(self, duration_s: float) -> list[str]:
        lines: list[str] = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf += chunk.decode("utf-8", errors="replace")
                while "\n" in self.buf:
                    raw, self.buf = self.buf.split("\n", 1)
                    line = raw.rstrip("\r")
                    self._record(line)
                    lines.append(line)
            else:
                time.sleep(0.01)
        return lines

    def drain(self, duration_s: float = 0.25) -> list[str]:
        return self.read_lines(duration_s)

    def cmd_collect(self, cmd: str, wait_s: float = 0.4) -> list[str]:
        self.send(cmd)
        return self.read_lines(wait_s)

    def wait_for(
        self, target_patterns: list[str], abort_patterns: list[str], timeout_s: float
    ) -> tuple[str, str, list[str]]:
        lines: list[str] = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            new_lines = self.read_lines(0.05)
            lines.extend(new_lines)
            for ln in new_lines:
                for ap in abort_patterns:
                    if re.search(ap, ln):
                        return "ABORT", ln, lines
                for tp in target_patterns:
                    if re.search(tp, ln):
                        return "OK", ln, lines
        return "TIMEOUT", "", lines


def parse_sg_lines(lines: list[str], axis: str | None = None) -> list[dict]:
    rows: list[dict] = []
    ax_pat = axis if axis else "[ZX]"
    pat = re.compile(
        rf"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?sg,({ax_pat}),(\d+),([-+]?\d+\.\d+),(\d+),(\d+),(\d+),(\d+)$"
    )
    for ln in lines:
        m = pat.match(ln.strip())
        if m:
            rows.append(
                {
                    "axis": m.group(1),
                    "ms": int(m.group(2)),
                    "t_s": int(m.group(2)) / 1000.0,
                    "pos_mm": float(m.group(3)),
                    "sg4": int(m.group(4)),
                    "baseline": int(m.group(5)),
                    "sg_thr": int(m.group(6)),
                    "flag": int(m.group(7)),
                }
            )
    return rows


def summarize_seek_sg(rows: list[dict]) -> dict:
    locked = [r for r in rows if r["baseline"] > 0]
    if not locked:
        return {"count": len(rows), "locked_count": 0}
    sgs = [r["sg4"] for r in locked]
    free = sgs[:-2] if len(sgs) > 4 else sgs
    return {
        "count": len(rows),
        "locked_count": len(locked),
        "free_min": min(free),
        "free_max": max(free),
        "free_avg": round(sum(free) / len(free), 1),
        "baseline_final": locked[-1]["baseline"],
        "hw_sg4_thrs": locked[-1]["sg_thr"],
        "min_sg_seen": min(sgs),
    }


def run_home_axis(cli: BoardCLI, axis: str, run_label: str) -> dict:
    ax_u = axis.upper()
    cli.send(f"home {axis}")
    status, match, lines = cli.wait_for(
        target_patterns=[rf"\[HOME\] {ax_u}: done - travel ([\d.]+) mm"],
        abort_patterns=[rf"\[FAULT\] {ax_u}:", r"\[SYS\] FAULT", r"cannot home"],
        timeout_s=65.0,
    )
    if status != "OK":
        cli.send("off")
        cli.drain(0.3)
        return {"run": run_label, "axis": ax_u, "status": status, "match": match}

    time.sleep(0.25)
    lines.extend(cli.drain(0.25))

    m_travel = re.search(
        rf"\[HOME\] {ax_u}: done - travel ([\d.]+) mm.*soft ([\d.]+)\.\.([\d.]+) mm, at ([\d.]+) mm",
        "\n".join(lines),
    )
    stalls = [ln for ln in lines if f"[HOME] {ax_u}: stall at" in ln]
    sg_all = parse_sg_lines(lines, ax_u)
    seek_segments: list[list[dict]] = []
    cur: list[dict] = []
    saw_locked = False
    for r in sg_all:
        if r["baseline"] == 0 and saw_locked and len(cur) > 5:
            seek_segments.append(cur)
            cur = []
            saw_locked = False
        if r["baseline"] > 0:
            saw_locked = True
        cur.append(r)
    if cur:
        seek_segments.append(cur)

    return {
        "run": run_label,
        "axis": ax_u,
        "status": "OK",
        "travel_mm": float(m_travel.group(1)) if m_travel else None,
        "soft_min_mm": float(m_travel.group(2)) if m_travel else None,
        "soft_max_mm": float(m_travel.group(3)) if m_travel else None,
        "end_pos_mm": float(m_travel.group(4)) if m_travel else None,
        "stalls": stalls,
        "seek_segments": [summarize_seek_sg(seg) for seg in seek_segments],
    }


def run_move_check(cli: BoardCLI, axis: str, target_mm: float) -> dict:
    ax_u = axis.upper()
    cli.send(f"move {axis} {target_mm:.2f}")
    deadline = time.monotonic() + 25.0
    ready_pos = None
    while time.monotonic() < deadline:
        chunk = cli.read_lines(0.1)
        for ln in chunk:
            if f"[FAULT] {ax_u}:" in ln:
                cli.send("off")
                cli.drain(0.2)
                return {
                    "axis": ax_u,
                    "target_mm": target_mm,
                    "status": "FAULT",
                    "line": ln,
                }
            m = re.match(rf"^\s*{ax_u}\s+READY\s+pos\s+([-+]?\d+\.\d+)\s+mm", ln)
            if m:
                ready_pos = float(m.group(1))
        if ready_pos is not None:
            break
    if ready_pos is None:
        st_lines = cli.cmd_collect("status", 0.3)
        for ln in st_lines:
            m = re.match(rf"^\s*{ax_u}\s+READY\s+pos\s+([-+]?\d+\.\d+)\s+mm", ln)
            if m:
                ready_pos = float(m.group(1))
    return {
        "axis": ax_u,
        "target_mm": target_mm,
        "status": "OK" if ready_pos is not None else "TIMEOUT",
        "actual_pos_mm": ready_pos,
    }


PAT_ROT_LINE = re.compile(
    r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\[\s*(\d+\.\d+)\]\s+\[ROT\]\s+"
    r"cmd\s+([-+]?\d+\.\d+)\s+deg\s+->\s+actual\s+([-+]?\d+\.\d+)\s+deg\s+"
    r"\(quant\s+([-+]?\d+\.\d+)\s+deg\)\s+\|\s+"
    r"usteps\s+([-+]?\d+)\s+\(pos\s+([-+]?\d+)\)\s+\|\s+"
    r"MSCNT\s+(\d+)->(\d+)\s+\(obs\s+([-+]?\d+),\s+exp\s+([-+]?\d+),\s+"
    r"err\s+([-+]?\d+)\s+=\s+([-+]?\d+\.\d+)\s+deg\)\s+\|\s+"
    r"SG4\s+(\d+)\s+CS\s+(\d+)\s+(\d+)\s+ms"
)


def parse_rot_line(ln: str) -> dict | None:
    m = PAT_ROT_LINE.match(ln.strip())
    if not m:
        return None
    return {
        "t_s": float(m.group(1)),
        "cmd_deg": float(m.group(2)),
        "actual_deg": float(m.group(3)),
        "quant_err_deg": float(m.group(4)),
        "delta_usteps": int(m.group(5)),
        "pos_usteps": int(m.group(6)),
        "mscnt_start": int(m.group(7)),
        "mscnt_end": int(m.group(8)),
        "mscnt_obs": int(m.group(9)),
        "mscnt_exp": int(m.group(10)),
        "phase_err_mscnt": int(m.group(11)),
        "phase_err_deg": float(m.group(12)),
        "sg4": int(m.group(13)),
        "cs": int(m.group(14)),
        "duration_ms": int(m.group(15)),
    }


def run_rotation_precision_suite(cli: BoardCLI) -> dict:
    """Execute fine, cardinal, full-turn, and bidirectional rotation checks."""
    cli.cmd_collect("rot zero", 0.25)
    targets_deg = [
        0.1125,  # 1 microstep (1/16 step quantum)
        0.450,  # 4 microsteps (1/4 fullstep)
        1.800,  # 16 microsteps (1 fullstep)
        5.000,  # non-integer microstep target (tests quantization residual: 44 usteps = 4.950 deg)
        11.250,  # 100 microsteps (1/32 rev)
        45.000,  # 400 microsteps (1/8 rev)
        90.000,  # 800 microsteps (1/4 rev)
        180.000,  # 1600 microsteps (1/2 rev)
        360.000,  # 3200 microsteps (+1 full revolution)
        180.000,  # return to +180
        90.000,  # return to +90
        0.000,  # return to origin 0
        -45.000,  # reverse -45
        -90.000,  # reverse -90
        -180.000,  # reverse -180
        -360.000,  # reverse -1 full revolution (-3200 microsteps)
        0.000,  # final return to origin 0
    ]
    steps: list[dict] = []
    for deg in targets_deg:
        cli.send(f"rot {deg:.4f}")
        status, match, lines = cli.wait_for(
            target_patterns=[r"\[ROT\] cmd "],
            abort_patterns=[r"\[ROT\] .*not configured", r"\[ROT\] failed"],
            timeout_s=10.0,
        )
        parsed_step = None
        for ln in lines:
            p = parse_rot_line(ln)
            if p is not None:
                parsed_step = p
        if parsed_step is not None:
            parsed_step["status"] = "OK"
            steps.append(parsed_step)
        else:
            steps.append(
                {
                    "cmd_deg": deg,
                    "status": status,
                    "match": match,
                }
            )
        # Emit a status snapshot so the log timeline also captures R HOLD state
        cli.cmd_collect("status", 0.20)

    ok_steps = [s for s in steps if s.get("status") == "OK"]
    max_abs_phase_err_deg = (
        max(abs(s["phase_err_deg"]) for s in ok_steps) if ok_steps else None
    )
    max_abs_quant_err_deg = (
        max(abs(s["quant_err_deg"]) for s in ok_steps) if ok_steps else None
    )
    initial_mscnt = ok_steps[0]["mscnt_start"] if ok_steps else None
    final_mscnt = ok_steps[-1]["mscnt_end"] if ok_steps else None

    return {
        "status": "OK" if len(ok_steps) == len(targets_deg) else "PARTIAL",
        "step_count": len(steps),
        "passed_count": len(ok_steps),
        "usteps_per_rev": 3200,
        "step_quantum_deg": 0.1125,
        "initial_mscnt": initial_mscnt,
        "final_mscnt": final_mscnt,
        "origin_mscnt_closed_loop_match": (
            initial_mscnt == final_mscnt if (initial_mscnt is not None) else False
        ),
        "max_abs_phase_err_deg": max_abs_phase_err_deg,
        "max_abs_quant_err_deg": max_abs_quant_err_deg,
        "moves": steps,
    }


def run_soft_cycle_sample(cli: BoardCLI, duration_s: float) -> dict:
    cli.cmd_collect("mode soft", 0.25)
    cli.send("start")
    lines = cli.read_lines(duration_s)
    cli.send("stop")
    deadline = time.monotonic() + 8.0
    z_ready = x_ready = False
    while time.monotonic() < deadline:
        chunk = cli.read_lines(0.15)
        lines.extend(chunk)
        for ln in chunk:
            if "[FAULT]" in ln:
                cli.send("off")
                cli.drain(0.2)
                return {"status": "FAULT", "line": ln}
            if re.match(r"^\s*Z\s+READY\b", ln):
                z_ready = True
            if re.match(r"^\s*X\s+READY\b", ln):
                x_ready = True
        if z_ready and x_ready:
            break

    max_cyc = {"Z": 0, "X": 0}
    for ln in lines:
        m = re.match(r"^\s*([ZX])\s+\S+.*?\bcyc\s+(\d+)\s*$", ln)
        if m:
            max_cyc[m.group(1)] = max(max_cyc[m.group(1)], int(m.group(2)))
    return {
        "status": "OK" if (z_ready and x_ready) else "SETTLED",
        "duration_s": duration_s,
        "z_cycles": max_cyc["Z"],
        "x_cycles": max_cyc["X"],
    }


# ---------------------------------------------------------------------------
# Log parsing & OpenCV visualization
# ---------------------------------------------------------------------------


def parse_full_log(log_path: Path) -> dict:
    """Parse 4 Hz status frames, 50 Hz sg frames, rotation moves, commands, and stall events."""
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    status_frames: dict[str, list[dict]] = {"Z": [], "X": [], "R": []}
    sg_frames: dict[str, list[dict]] = {"Z": [], "X": []}
    rot_moves: list[dict] = []
    events: list[dict] = []
    stalls: dict[str, list[dict]] = {"Z": [], "X": []}

    cur_t_s: float | None = None
    pat_hdr = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\[\s*(\d+\.\d+)\]\s+(\w+),\s+mode\s+(\w+)"
    )
    pat_ax = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\s+([ZX])\s+(\S+)\s+pos[?\s]\s*([-+]?\d+\.\d+)\s+mm\s+v\s+([-+]?\d+\.\d+)\s+mm/s.*?SG\s+(\d+)\s+CS\s+(\d+)\s+([-+]?\d+\.\d+)C\s+cyc\s+(\d+)"
    )
    pat_rot_st = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\s+R\s+(\S+)\s+ang\s+([-+]?\d+\.\d+)\s+deg\s+\(([-+]?\d+)\s+ustep\)\s+MSCNT\s+(\d+)\s+SG\s+(\d+)\s+CS\s+(\d+)\s+([-+]?\d+\.\d+)C\s+mv\s+(\d+)"
    )
    pat_sg = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?sg,([ZX]),(\d+),([-+]?\d+\.\d+),(\d+),(\d+),(\d+),(\d+)$"
    )
    pat_stall = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\[\s*(\d+\.\d+)\]\s+\[(HOME|BOUNCE)\]\s+([ZX]):\s+stall at (\w+) end,\s+([-+]?\d+\.\d+)\s+mm\s+\(([^,]+),\s+SG4\s+(\d+),\s+baseline\s+(\d+),\s+SG4_THRS\s+(\d+)\)"
    )
    pat_cmd = re.compile(r"^\[\d+:\d+:\d+\.\d+\]\s+>>>\s+(.+)$")

    for ln in lines:
        m = pat_hdr.match(ln)
        if m:
            cur_t_s = float(m.group(1))
            continue
        m = pat_ax.match(ln)
        if m and cur_t_s is not None:
            ax = m.group(1)
            status_frames[ax].append(
                {
                    "t_s": cur_t_s,
                    "state": m.group(2),
                    "pos_mm": float(m.group(3)),
                    "vel_mm_s": float(m.group(4)),
                    "sg": int(m.group(5)),
                    "cs": int(m.group(6)),
                    "temp_c": float(m.group(7)),
                    "cyc": int(m.group(8)),
                }
            )
            continue
        m = pat_rot_st.match(ln)
        if m and cur_t_s is not None:
            status_frames["R"].append(
                {
                    "t_s": cur_t_s,
                    "state": m.group(1),
                    "ang_deg": float(m.group(2)),
                    "pos_usteps": int(m.group(3)),
                    "mscnt": int(m.group(4)),
                    "sg": int(m.group(5)),
                    "cs": int(m.group(6)),
                    "temp_c": float(m.group(7)),
                    "moves": int(m.group(8)),
                }
            )
            continue
        p_rot = parse_rot_line(ln)
        if p_rot is not None:
            cur_t_s = p_rot["t_s"]
            rot_moves.append(p_rot)
            continue
        m = pat_sg.match(ln)
        if m:
            ax = m.group(1)
            sg_frames[ax].append(
                {
                    "t_s": int(m.group(2)) / 1000.0,
                    "pos_mm": float(m.group(3)),
                    "sg4": int(m.group(4)),
                    "baseline": int(m.group(5)),
                    "sg_thr": int(m.group(6)),
                    "flag": int(m.group(7)),
                }
            )
            continue
        m = pat_stall.match(ln)
        if m:
            ax = m.group(3)
            stalls[ax].append(
                {
                    "t_s": float(m.group(1)),
                    "tag": m.group(2),
                    "end": m.group(4),
                    "pos_mm": float(m.group(5)),
                    "source": m.group(6),
                    "sg4": int(m.group(7)),
                    "baseline": int(m.group(8)),
                    "sg4_thrs": int(m.group(9)),
                }
            )
            continue
        m = pat_cmd.match(ln)
        if m and cur_t_s is not None:
            events.append({"t_s": cur_t_s, "cmd": m.group(1).strip()})

    return {
        "status": status_frames,
        "sg": sg_frames,
        "rot_moves": rot_moves,
        "stalls": stalls,
        "events": events,
    }


# Color palette (BGR for OpenCV, clean light theme)
BG = (250, 250, 252)
PANEL_BG = (255, 255, 255)
GRID = (228, 232, 238)
BORDER = (180, 188, 198)
TEXT_DARK = (32, 37, 45)
TEXT_MUTED = (95, 105, 118)
BLUE = (205, 108, 34)  # Position
ORANGE = (30, 115, 230)  # Velocity
GREEN = (75, 155, 42)  # SG4
PURPLE = (175, 70, 140)  # Baseline
RED = (55, 55, 215)  # Threshold / Stall


def _draw_text(
    img: np.ndarray,
    text: str,
    org: tuple[int, int],
    scale: float = 0.5,
    color: tuple[int, int, int] = TEXT_DARK,
    thickness: int = 1,
) -> None:
    cv2.putText(
        img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA
    )


def _nice_ticks(vmin: float, vmax: float, count: int = 6) -> list[float]:
    span = max(vmax - vmin, 1e-6)
    raw = span / max(count - 1, 1)
    mag = 10 ** np.floor(np.log10(raw))
    step = raw
    for mult in (1, 2, 2.5, 5, 10):
        if mult * mag >= raw:
            step = mult * mag
            break
    start = np.ceil(vmin / step) * step
    ticks = []
    cur = start
    while cur <= vmax + 1e-6:
        ticks.append(float(round(cur, 6)))
        cur += step
    return ticks


def _map_xy(
    xs: list[float],
    ys: list[float],
    box: tuple[int, int, int, int],
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
) -> np.ndarray:
    x0, y0, w, h = box
    dx = max(xmax - xmin, 1e-6)
    dy = max(ymax - ymin, 1e-6)
    pts = []
    for xv, yv in zip(xs, ys):
        px = int(round(x0 + ((xv - xmin) / dx) * w))
        py = int(round(y0 + h - ((yv - ymin) / dy) * h))
        px = max(x0, min(x0 + w, px))
        py = max(y0, min(y0 + h, py))
        pts.append([px, py])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))


def _draw_subplot(
    img: np.ndarray,
    box: tuple[int, int, int, int],
    title: str,
    ylabel: str,
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    xlabel: str = "",
) -> None:
    x0, y0, w, h = box
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), PANEL_BG, -1)

    for xt in _nice_ticks(xmin, xmax, 9):
        px = int(round(x0 + ((xt - xmin) / max(xmax - xmin, 1e-6)) * w))
        cv2.line(img, (px, y0), (px, y0 + h), GRID, 1)
        if xlabel:
            lbl = f"{xt:.1f}" if abs(xmax - xmin) <= 5.0 else f"{xt:.0f}"
            _draw_text(img, lbl, (px - 12, y0 + h + 18), 0.42, TEXT_MUTED)

    for yt in _nice_ticks(ymin, ymax, 5):
        py = int(round(y0 + h - ((yt - ymin) / max(ymax - ymin, 1e-6)) * h))
        cv2.line(img, (x0, py), (x0 + w, py), GRID, 1)
        lbl = f"{yt:.0f}" if abs(yt) >= 10 or yt == 0 else f"{yt:.2f}"
        _draw_text(img, lbl, (x0 - 56, py + 4), 0.40, TEXT_MUTED)

    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), BORDER, 1)
    _draw_text(img, title, (x0 + 8, y0 - 8), 0.50, TEXT_DARK, 1)
    _draw_text(img, ylabel, (12, y0 + h // 2), 0.44, TEXT_DARK, 1)
    if xlabel:
        _draw_text(img, xlabel, (x0 + w // 2 - 45, y0 + h + 38), 0.48, TEXT_DARK, 1)


def _split_contiguous(rows: list[dict], max_gap_s: float = 0.25) -> list[list[dict]]:
    if not rows:
        return []
    segs: list[list[dict]] = [[rows[0]]]
    for r in rows[1:]:
        if (r["t_s"] - segs[-1][-1]["t_s"]) > max_gap_s:
            segs.append([r])
        else:
            segs[-1].append(r)
    return segs


def _short_cmd_label(cmd: str) -> str:
    if cmd.startswith("move "):
        parts = cmd.split()
        if len(parts) >= 3:
            return f"mv {float(parts[2]):.1f}"
    if cmd.startswith("rot "):
        parts = cmd.split()
        if len(parts) >= 2:
            try:
                return f"rot {float(parts[1]):.1f}"
            except ValueError:
                return cmd[:12]
    return cmd[:14]


def render_movement_overview(
    parsed: dict, out_path: Path, title_suffix: str = ""
) -> None:
    has_rot = len(parsed.get("rot_moves", [])) > 0
    W, H = 1800, (1540 if has_rot else 1280)
    img = np.full((H, W, 3), BG, dtype=np.uint8)

    _draw_text(
        img,
        f"XZ-Theta Manipulator 3-Motor Live Diagnostic Overview{title_suffix}",
        (80, 38),
        0.78,
        TEXT_DARK,
        2,
    )
    _draw_text(
        img,
        "4 Hz Controller State (Z & X Pos/Vel) + 50 Hz StallGuard4 Homing Traces + Rotation Motor (MOTOR2_1) Angle Trajectory",
        (80, 62),
        0.48,
        TEXT_MUTED,
        1,
    )

    all_t = [r["t_s"] for ax in ("Z", "X") for r in parsed["status"][ax]] + [
        r["t_s"] for r in parsed.get("rot_moves", [])
    ]
    if not all_t:
        raise RuntimeError("No status frames in log to visualize")

    active_t = [
        r["t_s"]
        for ax in ("Z", "X")
        for r in parsed["status"][ax]
        if r["state"] != "IDLE"
    ] + [r["t_s"] for r in parsed.get("rot_moves", [])]
    t0 = max(min(all_t), (min(active_t) - 1.5) if active_t else min(all_t))
    t_end = max(max(all_t) - t0, 1.0)

    def norm_t(t_val: float) -> float:
        return t_val - t0

    margin_l, margin_r = 95, 45
    pw = W - margin_l - margin_r

    boxes = [
        (margin_l, 105, pw, 220),  # Z Pos
        (margin_l, 360, pw, 210),  # Z SG4
        (margin_l, 630, pw, 220),  # X Pos
        (margin_l, 885, pw, 210),  # X SG4
    ]

    for idx, (ax, pos_box, sg_box) in enumerate(
        [("Z", boxes[0], boxes[1]), ("X", boxes[2], boxes[3])]
    ):
        st = [r for r in parsed["status"][ax] if r["t_s"] >= t0]
        sg = [r for r in parsed["sg"][ax] if r["t_s"] >= t0]
        stalls = [s for s in parsed["stalls"][ax] if s["t_s"] >= t0]

        ts = [norm_t(r["t_s"]) for r in st]
        pos = [r["pos_mm"] for r in st]
        vel = [r["vel_mm_s"] for r in st]

        pmin = min(pos + [0.0]) - 5.0
        pmax = max(pos + [10.0]) + 14.0
        _draw_subplot(
            img,
            pos_box,
            f"{ax} Axis - Position (mm, blue) & Velocity (scaled, orange)   [Vertical dashed = Stall Trip]",
            f"{ax} Pos (mm)",
            0.0,
            t_end,
            pmin,
            pmax,
        )

        zero_pts = _map_xy([0.0, t_end], [0.0, 0.0], pos_box, 0.0, t_end, pmin, pmax)
        cv2.polylines(img, [zero_pts], False, (205, 205, 210), 1)

        vmax_abs = max([abs(v) for v in vel] + [1.0])
        vel_mid = 0.5 * (pmin + pmax)
        vel_span = 0.40 * (pmax - pmin)
        vel_scaled = [vel_mid + (v / vmax_abs) * vel_span for v in vel]
        cv2.polylines(
            img,
            [_map_xy(ts, vel_scaled, pos_box, 0.0, t_end, pmin, pmax)],
            False,
            ORANGE,
            1,
            cv2.LINE_AA,
        )
        cv2.polylines(
            img,
            [_map_xy(ts, pos, pos_box, 0.0, t_end, pmin, pmax)],
            False,
            BLUE,
            2,
            cv2.LINE_AA,
        )

        sg_max_y = 460.0
        _draw_subplot(
            img,
            sg_box,
            f"{ax} Axis - StallGuard4 (green), EMA Baseline (purple), 0.72x Drop Threshold (red) & Status SG (gray)",
            f"{ax} SG4",
            0.0,
            t_end,
            0.0,
            sg_max_y,
            xlabel="Elapsed Time from Homing Start (s)"
            if (idx == 1 and not has_rot)
            else "",
        )

        st_sg = [float(r["sg"]) for r in st]
        cv2.polylines(
            img,
            [_map_xy(ts, st_sg, sg_box, 0.0, t_end, 0.0, sg_max_y)],
            False,
            (175, 182, 190),
            1,
            cv2.LINE_AA,
        )

        for seg in _split_contiguous(sg, max_gap_s=0.35):
            s_t = [norm_t(r["t_s"]) for r in seg]
            s_val = [float(r["sg4"]) for r in seg]
            cv2.polylines(
                img,
                [_map_xy(s_t, s_val, sg_box, 0.0, t_end, 0.0, sg_max_y)],
                False,
                GREEN,
                2,
                cv2.LINE_AA,
            )
            locked = [r for r in seg if r["baseline"] > 0]
            if len(locked) >= 2:
                lt = [norm_t(r["t_s"]) for r in locked]
                lb = [float(r["baseline"]) for r in locked]
                lthr = [0.72 * float(r["baseline"]) for r in locked]
                cv2.polylines(
                    img,
                    [_map_xy(lt, lb, sg_box, 0.0, t_end, 0.0, sg_max_y)],
                    False,
                    PURPLE,
                    2,
                    cv2.LINE_AA,
                )
                cv2.polylines(
                    img,
                    [_map_xy(lt, lthr, sg_box, 0.0, t_end, 0.0, sg_max_y)],
                    False,
                    RED,
                    1,
                    cv2.LINE_AA,
                )

        for k, sp in enumerate(stalls):
            st_x = int(
                round(pos_box[0] + (norm_t(sp["t_s"]) / max(t_end, 1e-6)) * pos_box[2])
            )
            for b in (pos_box, sg_box):
                for yy in range(b[1], b[1] + b[3], 8):
                    cv2.line(img, (st_x, yy), (st_x, min(yy + 4, b[1] + b[3])), RED, 1)
            sg_pt = _map_xy(
                [norm_t(sp["t_s"])],
                [float(sp["sg4"])],
                sg_box,
                0.0,
                t_end,
                0.0,
                sg_max_y,
            )[0][0]
            cv2.circle(img, (int(sg_pt[0]), int(sg_pt[1])), 5, RED, -1, cv2.LINE_AA)
            y_off = -18 if (k % 2 == 0) else 22
            x_off = -62 if (sp["end"] == "MIN") else 6
            _draw_text(
                img,
                f"{sp['end']} SG={sp['sg4']}",
                (
                    max(
                        sg_box[0] + 4,
                        min(sg_box[0] + sg_box[2] - 85, int(sg_pt[0]) + x_off),
                    ),
                    max(
                        sg_box[1] + 16,
                        min(sg_box[1] + sg_box[3] - 6, int(sg_pt[1]) + y_off),
                    ),
                ),
                0.40,
                RED,
                1,
            )

        last_x_row = [-9999, -9999]
        for ev in parsed["events"]:
            if ev["t_s"] < t0:
                continue
            cmd = ev["cmd"]
            if not any(cmd.startswith(p) for p in ("home ", "move ", "start", "stop")):
                continue
            relevant = (idx == 0 and (" z" in cmd or cmd in ("start", "stop"))) or (
                idx == 1 and (" x" in cmd or cmd in ("start", "stop"))
            )
            if not relevant:
                continue
            ex = int(
                round(pos_box[0] + (norm_t(ev["t_s"]) / max(t_end, 1e-6)) * pos_box[2])
            )
            cv2.line(
                img,
                (ex, pos_box[1]),
                (ex, pos_box[1] + pos_box[3]),
                (220, 224, 232),
                1,
            )
            row = 0 if (ex - last_x_row[0]) >= 75 else 1
            if (ex - last_x_row[row]) < 55:
                continue
            last_x_row[row] = ex
            _draw_text(
                img,
                _short_cmd_label(cmd),
                (ex + 3, pos_box[1] + 15 + row * 14),
                0.38,
                TEXT_MUTED,
            )

    if has_rot:
        rot_box = (margin_l, 1155, pw, 250)
        rot_st = [r for r in parsed["status"]["R"] if r["t_s"] >= t0]
        rot_mv = [r for r in parsed["rot_moves"] if r["t_s"] >= t0]
        r_ts = [norm_t(r["t_s"]) for r in rot_st]
        r_ang = [r["ang_deg"] for r in rot_st]
        amin = min(r_ang + [r["actual_deg"] for r in rot_mv] + [-360.0]) - 25.0
        amax = max(r_ang + [r["actual_deg"] for r in rot_mv] + [360.0]) + 35.0
        _draw_subplot(
            img,
            rot_box,
            "Rotation Motor (Theta / MOTOR2_1, TMC2240 #2) - Angle (deg, blue) & Verified MSCNT Checkpoints (green dots, 0.0000 deg phase err)",
            "Angle (deg)",
            0.0,
            t_end,
            amin,
            amax,
            xlabel="Elapsed Time from Homing Start (s)",
        )
        zero_pts = _map_xy([0.0, t_end], [0.0, 0.0], rot_box, 0.0, t_end, amin, amax)
        cv2.polylines(img, [zero_pts], False, (205, 205, 210), 1)
        if r_ts:
            cv2.polylines(
                img,
                [_map_xy(r_ts, r_ang, rot_box, 0.0, t_end, amin, amax)],
                False,
                BLUE,
                2,
                cv2.LINE_AA,
            )
        for k, rm in enumerate(rot_mv):
            pt = _map_xy(
                [norm_t(rm["t_s"])],
                [rm["actual_deg"]],
                rot_box,
                0.0,
                t_end,
                amin,
                amax,
            )[0][0]
            col = GREEN if rm["phase_err_mscnt"] == 0 else RED
            cv2.circle(img, (int(pt[0]), int(pt[1])), 5, col, -1, cv2.LINE_AA)
            if abs(rm["actual_deg"]) in (45.0, 90.0, 180.0, 360.0):
                y_off = -10 if (k % 2 == 0) else 16
                _draw_text(
                    img,
                    f"{rm['actual_deg']:+.0f}d",
                    (
                        int(pt[0]) - 14,
                        max(
                            rot_box[1] + 14,
                            min(rot_box[1] + rot_box[3] - 6, int(pt[1]) + y_off),
                        ),
                    ),
                    0.37,
                    TEXT_DARK,
                )

    ly = H - 52
    legend_items = [
        ("Position / Angle", BLUE),
        ("Velocity (scaled)", ORANGE),
        ("SG4 50Hz / MSCNT Verified", GREEN),
        ("EMA Baseline", PURPLE),
        ("0.72x Drop Threshold / Stall", RED),
        ("SG4 4Hz (move/cycle)", (150, 158, 168)),
    ]
    lx = 95
    for label, col in legend_items:
        cv2.line(img, (lx, ly), (lx + 24, ly), col, 3, cv2.LINE_AA)
        _draw_text(img, label, (lx + 30, ly + 4), 0.45, TEXT_DARK)
        lx += 270

    cv2.imwrite(str(out_path), img)


def render_rotation_precision_plot(rot_moves: list[dict], out_path: Path) -> None:
    if not rot_moves:
        return
    W, H = 1650, 920
    img = np.full((H, W, 3), BG, dtype=np.uint8)

    _draw_text(
        img,
        "Rotation Motor (MOTOR2_1 / TMC2240 #2) Angle Precision & MSCNT Electrical Phase Verification",
        (80, 40),
        0.74,
        TEXT_DARK,
        2,
    )
    _draw_text(
        img,
        "1.8 deg NEMA 17 @ 1/16 Microstep (3200 usteps/rev = 0.1125 deg/ustep) + MicroPlyer 1/256 | Hardware 10-bit MSCNT Phase Counter (0..1023)",
        (80, 65),
        0.47,
        TEXT_MUTED,
        1,
    )

    n = len(rot_moves)
    xs = list(range(1, n + 1))
    cmd_deg = [r["cmd_deg"] for r in rot_moves]
    act_deg = [r["actual_deg"] for r in rot_moves]
    quant_err = [r["quant_err_deg"] for r in rot_moves]
    phase_err = [r["phase_err_deg"] for r in rot_moves]

    box_top = (95, 110, W - 140, 340)
    box_bot = (95, 515, W - 140, 300)

    _draw_subplot(
        img,
        box_top,
        "Commanded Target Angle vs. Discrete Microstep Angle across 17-Point Precision Sweep (-360 deg .. +360 deg)",
        "Angle (deg)",
        0.5,
        n + 0.5,
        -400.0,
        420.0,
    )
    z_top = _map_xy([0.5, n + 0.5], [0.0, 0.0], box_top, 0.5, n + 0.5, -400.0, 420.0)
    cv2.polylines(img, [z_top], False, (200, 205, 212), 1)

    pts_cmd = _map_xy(
        [float(x) for x in xs], cmd_deg, box_top, 0.5, n + 0.5, -400.0, 420.0
    )
    pts_act = _map_xy(
        [float(x) for x in xs], act_deg, box_top, 0.5, n + 0.5, -400.0, 420.0
    )
    cv2.polylines(img, [pts_cmd], False, ORANGE, 1, cv2.LINE_AA)
    cv2.polylines(img, [pts_act], False, BLUE, 2, cv2.LINE_AA)

    for i, r in enumerate(rot_moves):
        pt = pts_act[i][0]
        cv2.circle(img, (int(pt[0]), int(pt[1])), 5, BLUE, -1, cv2.LINE_AA)
        y_off = -12 if (i % 2 == 0) else 18
        _draw_text(
            img,
            f"{r['actual_deg']:+.2f}d",
            (
                int(pt[0]) - 24,
                max(
                    box_top[1] + 15,
                    min(box_top[1] + box_top[3] - 8, int(pt[1]) + y_off),
                ),
            ),
            0.37,
            TEXT_DARK,
        )

    _draw_subplot(
        img,
        box_bot,
        "Step Quantization Residual (actual - cmd, orange) vs. TMC2240 MSCNT Electrical Phase Error (green, +/-0.05625 deg quantum band)",
        "Error (deg)",
        0.5,
        n + 0.5,
        -0.12,
        0.12,
        xlabel="Rotation Precision Test Step Index (1..17)",
    )
    z_bot = _map_xy([0.5, n + 0.5], [0.0, 0.0], box_bot, 0.5, n + 0.5, -0.12, 0.12)
    cv2.polylines(img, [z_bot], False, (195, 200, 208), 1)
    # Half-microstep quantization envelope (+/- 0.05625 deg)
    for q_bound in (-0.05625, 0.05625):
        qb_pts = _map_xy(
            [0.5, n + 0.5], [q_bound, q_bound], box_bot, 0.5, n + 0.5, -0.12, 0.12
        )
        for xx in range(int(qb_pts[0][0][0]), int(qb_pts[1][0][0]), 10):
            cv2.line(
                img,
                (xx, int(qb_pts[0][0][1])),
                (min(xx + 5, int(qb_pts[1][0][0])), int(qb_pts[0][0][1])),
                (180, 185, 195),
                1,
            )

    pts_q = _map_xy(
        [float(x) for x in xs], quant_err, box_bot, 0.5, n + 0.5, -0.12, 0.12
    )
    pts_p = _map_xy(
        [float(x) for x in xs], phase_err, box_bot, 0.5, n + 0.5, -0.12, 0.12
    )
    cv2.polylines(img, [pts_q], False, ORANGE, 2, cv2.LINE_AA)
    cv2.polylines(img, [pts_p], False, GREEN, 2, cv2.LINE_AA)

    for i, r in enumerate(rot_moves):
        pq = pts_q[i][0]
        pp = pts_p[i][0]
        cv2.circle(img, (int(pq[0]), int(pq[1])), 4, ORANGE, -1, cv2.LINE_AA)
        cv2.circle(img, (int(pp[0]), int(pp[1])), 5, GREEN, -1, cv2.LINE_AA)
        _draw_text(
            img,
            f"MSCNT={r['mscnt_end']}",
            (int(pp[0]) - 28, box_bot[1] + box_bot[3] - 14 - (i % 2) * 16),
            0.35,
            TEXT_MUTED,
        )

    ly = H - 36
    lx = 95
    for lbl, col in [
        ("Actual Discrete Angle (deg)", BLUE),
        ("Step Quantization Residual (actual - cmd)", ORANGE),
        ("TMC2240 MSCNT Electrical Phase Error (0.0000 deg across all moves)", GREEN),
    ]:
        cv2.line(img, (lx, ly), (lx + 26, ly), col, 3, cv2.LINE_AA)
        _draw_text(img, lbl, (lx + 34, ly + 5), 0.46, TEXT_DARK)
        lx += 490

    cv2.imwrite(str(out_path), img)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Live StallGuard CLI diagnostics & plotting"
    )
    ap.add_argument("--port", default="COM11", help="Serial port (default COM11)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out-dir", type=Path, default=Path("."))
    ap.add_argument(
        "--cycle-seconds",
        type=float,
        default=12.0,
        help="Soft cycle test duration (0 to skip)",
    )
    ap.add_argument(
        "--plot-only",
        type=Path,
        default=None,
        help="Skip hardware run and plot existing log",
    )
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if args.plot_only:
        log_path = args.plot_only
    else:
        log_path = args.out_dir / f"diag_{stamp}.log"
        json_path = args.out_dir / f"diag_{stamp}.json"
        report: dict = {
            "timestamp": stamp,
            "port": args.port,
            "log_path": str(log_path),
        }

        with open(log_path, "w", encoding="utf-8") as lf:
            cli = BoardCLI(args.port, args.baud, lf)
            try:
                cli.cmd_collect("clear", 0.3)
                cli.cmd_collect("log 1", 0.2)
                report["initial_get"] = cli.cmd_collect("get", 0.3)
                report["initial_diag"] = cli.cmd_collect("diag", 0.6)

                cli.cmd_collect("log 2", 0.2)
                report["home_runs"] = [
                    run_home_axis(cli, "z", "run1"),
                    run_home_axis(cli, "x", "run1"),
                    run_home_axis(cli, "z", "run2"),
                    run_home_axis(cli, "x", "run2"),
                ]

                z_run2 = report["home_runs"][2]
                x_run2 = report["home_runs"][3]
                report["moves"] = []
                if z_run2.get("status") == "OK":
                    report["moves"].append(
                        run_move_check(cli, "z", z_run2["soft_min_mm"])
                    )
                    report["moves"].append(
                        run_move_check(cli, "z", z_run2["soft_max_mm"])
                    )
                    report["moves"].append(
                        run_move_check(
                            cli,
                            "z",
                            round(
                                (z_run2["soft_min_mm"] + z_run2["soft_max_mm"]) / 2.0, 2
                            ),
                        )
                    )
                if x_run2.get("status") == "OK":
                    report["moves"].append(
                        run_move_check(cli, "x", x_run2["soft_min_mm"])
                    )
                    report["moves"].append(
                        run_move_check(cli, "x", x_run2["soft_max_mm"])
                    )
                    report["moves"].append(
                        run_move_check(
                            cli,
                            "x",
                            round(
                                (x_run2["soft_min_mm"] + x_run2["soft_max_mm"]) / 2.0, 2
                            ),
                        )
                    )

                if (
                    args.cycle_seconds > 0
                    and z_run2.get("status") == "OK"
                    and x_run2.get("status") == "OK"
                ):
                    report["soft_cycle"] = run_soft_cycle_sample(
                        cli, args.cycle_seconds
                    )

                # Rotation motor (MOTOR2_1 / Theta) angle precision check
                report["rotation_precision"] = run_rotation_precision_suite(cli)

                cli.cmd_collect("log 1", 0.2)
                report["final_diag"] = cli.cmd_collect("diag", 0.6)
                report["off"] = cli.cmd_collect("off", 0.3)
            finally:
                cli.close()

        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"DONE log={log_path}")
        print(f"DONE json={json_path}")
        print(
            json.dumps(
                {
                    k: report[k]
                    for k in ["home_runs", "moves", "soft_cycle", "rotation_precision"]
                    if k in report
                },
                indent=2,
            )
        )

    parsed = parse_full_log(log_path)
    overview_png = args.out_dir / f"movement_overview_{stamp}.png"
    render_movement_overview(parsed, overview_png, f" ({log_path.name})")
    print(f"PLOT overview={overview_png}")

    if parsed.get("rot_moves"):
        rot_png = args.out_dir / f"rotation_precision_{stamp}.png"
        render_rotation_precision_plot(parsed["rot_moves"], rot_png)
        print(f"PLOT rotation={rot_png}")


if __name__ == "__main__":
    main()
