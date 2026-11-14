"""Automated live diagnostic runner & telemetry visualizer for stallguard_cli.

Runs homing repeatability (2x), soft-limit/midpoint positioning, and optional
soft-cycle checks over USB CDC CLI, logs raw + structured JSON output, and
renders high-resolution PNG movement & StallGuard4 plots (OpenCV + NumPy).
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
    """Parse 4 Hz status frames, 50 Hz sg frames, commands, and stall events."""
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    status_frames: dict[str, list[dict]] = {"Z": [], "X": []}
    sg_frames: dict[str, list[dict]] = {"Z": [], "X": []}
    events: list[dict] = []
    stalls: dict[str, list[dict]] = {"Z": [], "X": []}

    cur_t_s: float | None = None
    pat_hdr = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\[\s*(\d+\.\d+)\]\s+(\w+),\s+mode\s+(\w+)"
    )
    pat_ax = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\s+([ZX])\s+(\S+)\s+pos[?\s]\s*([-+]?\d+\.\d+)\s+mm\s+v\s+([-+]?\d+\.\d+)\s+mm/s.*?SG\s+(\d+)\s+CS\s+(\d+)\s+([-+]?\d+\.\d+)C\s+cyc\s+(\d+)"
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
        lbl = f"{yt:.0f}" if abs(yt) >= 10 or yt == 0 else f"{yt:.1f}"
        _draw_text(img, lbl, (x0 - 52, py + 4), 0.42, TEXT_MUTED)

    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), BORDER, 1)
    _draw_text(img, title, (x0 + 8, y0 - 8), 0.50, TEXT_DARK, 1)
    _draw_text(img, ylabel, (14, y0 + h // 2), 0.46, TEXT_DARK, 1)
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
    return cmd[:14]


def render_movement_overview(
    parsed: dict, out_path: Path, title_suffix: str = ""
) -> None:
    W, H = 1800, 1280
    img = np.full((H, W, 3), BG, dtype=np.uint8)

    _draw_text(
        img,
        f"XZ-Manipulator StallGuard CLI Live Diagnostic Overview{title_suffix}",
        (80, 38),
        0.78,
        TEXT_DARK,
        2,
    )
    _draw_text(
        img,
        "4 Hz Controller State (Pos & Vel) + 50 Hz StallGuard4 Trace + Homing End-Stop Stall Trips",
        (80, 62),
        0.48,
        TEXT_MUTED,
        1,
    )

    all_t = [r["t_s"] for ax in ("Z", "X") for r in parsed["status"][ax]]
    if not all_t:
        raise RuntimeError("No status frames in log to visualize")

    active_t = [
        r["t_s"]
        for ax in ("Z", "X")
        for r in parsed["status"][ax]
        if r["state"] != "IDLE"
    ]
    t0 = max(min(all_t), (min(active_t) - 1.5) if active_t else min(all_t))
    t_end = max(max(all_t) - t0, 1.0)

    def norm_t(t_val: float) -> float:
        return t_val - t0

    margin_l, margin_r = 95, 45
    pw = W - margin_l - margin_r

    boxes = [
        (margin_l, 105, pw, 225),  # Z Pos
        (margin_l, 365, pw, 215),  # Z SG4
        (margin_l, 645, pw, 225),  # X Pos
        (margin_l, 905, pw, 215),  # X SG4
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
            xlabel="Elapsed Time from Homing Start (s)" if idx == 1 else "",
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

        # Mark stall events on both panels with staggered labels
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

        # Command phase annotations with 2-row anti-overlap stagger
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

    ly = H - 62
    legend_items = [
        ("Position (mm)", BLUE),
        ("Velocity (scaled mm/s)", ORANGE),
        ("SG4 50Hz (seek/bounce)", GREEN),
        ("EMA Baseline", PURPLE),
        ("0.72x Drop Threshold / Stall", RED),
        ("SG4 4Hz (move/cycle)", (150, 158, 168)),
    ]
    lx = 95
    for label, col in legend_items:
        cv2.line(img, (lx, ly), (lx + 24, ly), col, 3, cv2.LINE_AA)
        _draw_text(img, label, (lx + 30, ly + 4), 0.45, TEXT_DARK)
        lx += 265

    cv2.imwrite(str(out_path), img)


def _extract_z_max_approach(
    parsed: dict, window_s: float = 0.55
) -> tuple[list[dict], dict] | None:
    z_stalls = [s for s in parsed["stalls"]["Z"] if s["end"] == "MAX"]
    if not z_stalls:
        return None
    stall = z_stalls[-1]
    t_stall = stall["t_s"]
    rows = [
        r
        for r in parsed["sg"]["Z"]
        if (t_stall - window_s) <= r["t_s"] <= (t_stall + 0.05) and r["baseline"] > 0
    ]
    if not rows:
        return None
    return rows, stall


def render_z_max_comparison(old_parsed: dict, new_parsed: dict, out_path: Path) -> None:
    old_ext = _extract_z_max_approach(old_parsed)
    new_ext = _extract_z_max_approach(new_parsed)
    if old_ext is None or new_ext is None:
        return
    old_rows, old_stall = old_ext
    new_rows, new_stall = new_ext

    W, H = 1500, 760
    img = np.full((H, W, 3), BG, dtype=np.uint8)
    _draw_text(
        img,
        "Z Axis MAX End-Stop Approach: Baseline Chase Fix (ema_shift 5 -> 7)",
        (75, 40),
        0.74,
        TEXT_DARK,
        2,
    )
    _draw_text(
        img,
        "Aligned to start of SG4 load ramp (last free-run sample >= 328) so ramp delay before stall trip is directly visible",
        (75, 65),
        0.46,
        TEXT_MUTED,
        1,
    )

    def align_to_ramp(rows: list[dict]) -> tuple[list[float], list[dict]]:
        ramp_idx = 0
        for i, r in enumerate(rows):
            if r["sg4"] >= 328:
                ramp_idx = i
        t0 = rows[ramp_idx]["t_s"]
        return [r["t_s"] - t0 for r in rows], rows

    old_t, old_r = align_to_ramp(old_rows)
    new_t, new_r = align_to_ramp(new_rows)
    xmin = min(old_t[0], new_t[0], -0.25)
    xmax = max(old_t[-1], new_t[-1], 0.25)

    boxes = [
        (95, 115, 620, 520),
        (815, 115, 620, 520),
    ]
    panels = [
        ("BEFORE: ema_shift = 5 (tau ~ 64 ms)", old_t, old_r, old_stall, boxes[0]),
        ("AFTER:  ema_shift = 7 (tau ~ 256 ms)", new_t, new_r, new_stall, boxes[1]),
    ]
    for title, ts, rows, stall, box in panels:
        _draw_subplot(
            img,
            box,
            title,
            "SG4 Value",
            xmin,
            xmax,
            100.0,
            380.0,
            xlabel="Time Relative to Ramp Start (s)",
        )
        z_pts = _map_xy([0.0, 0.0], [100.0, 380.0], box, xmin, xmax, 100.0, 380.0)
        cv2.polylines(img, [z_pts], False, (195, 200, 210), 1)

        sgs = [float(r["sg4"]) for r in rows]
        bas = [float(r["baseline"]) for r in rows]
        thr = [0.72 * float(r["baseline"]) for r in rows]

        cv2.polylines(
            img,
            [_map_xy(ts, sgs, box, xmin, xmax, 100.0, 380.0)],
            False,
            GREEN,
            2,
            cv2.LINE_AA,
        )
        cv2.polylines(
            img,
            [_map_xy(ts, bas, box, xmin, xmax, 100.0, 380.0)],
            False,
            PURPLE,
            2,
            cv2.LINE_AA,
        )
        cv2.polylines(
            img,
            [_map_xy(ts, thr, box, xmin, xmax, 100.0, 380.0)],
            False,
            RED,
            2,
            cv2.LINE_AA,
        )

        t_trip = stall["t_s"] - (rows[0]["t_s"] - ts[0])
        pt = _map_xy([t_trip], [float(stall["sg4"])], box, xmin, xmax, 100.0, 380.0)[0][
            0
        ]
        cv2.circle(img, (int(pt[0]), int(pt[1])), 7, RED, -1, cv2.LINE_AA)
        _draw_text(
            img,
            f"TRIP +{t_trip * 1000:.0f} ms: SG4={stall['sg4']}, base={stall['baseline']}",
            (box[0] + 18, box[1] + 465),
            0.48,
            RED,
            1,
        )
        _draw_text(
            img,
            f"0.72 * baseline = {0.72 * stall['baseline']:.1f}",
            (box[0] + 18, box[1] + 490),
            0.46,
            TEXT_MUTED,
            1,
        )

    ly = H - 36
    lx = 95
    for lbl, col in [
        ("SG4 (50 Hz)", GREEN),
        ("EMA Baseline", PURPLE),
        ("0.72x Drop Threshold & Trip Point", RED),
    ]:
        cv2.line(img, (lx, ly), (lx + 26, ly), col, 3, cv2.LINE_AA)
        _draw_text(img, lbl, (lx + 34, ly + 5), 0.48, TEXT_DARK)
        lx += 330

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
        "--compare-log",
        type=Path,
        default=None,
        help="Optional prior log for Z MAX before/after plot",
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
                report["initial_diag"] = cli.cmd_collect("diag", 0.5)

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

                cli.cmd_collect("log 1", 0.2)
                report["final_diag"] = cli.cmd_collect("diag", 0.5)
                report["off"] = cli.cmd_collect("off", 0.3)
            finally:
                cli.close()

        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"DONE log={log_path}")
        print(f"DONE json={json_path}")
        print(
            json.dumps(
                {k: report[k] for k in ["home_runs", "moves"] if k in report}, indent=2
            )
        )
        if "soft_cycle" in report:
            print("soft_cycle:", json.dumps(report["soft_cycle"]))

    parsed = parse_full_log(log_path)
    overview_png = args.out_dir / f"movement_overview_{stamp}.png"
    render_movement_overview(parsed, overview_png, f" ({log_path.name})")
    print(f"PLOT overview={overview_png}")

    if args.compare_log and args.compare_log.exists():
        old_parsed = parse_full_log(args.compare_log)
        cmp_png = args.out_dir / f"z_max_stall_comparison_{stamp}.png"
        render_z_max_comparison(old_parsed, parsed, cmp_png)
        print(f"PLOT comparison={cmp_png}")


if __name__ == "__main__":
    main()
