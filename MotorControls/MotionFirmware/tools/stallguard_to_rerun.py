# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "rerun-sdk==0.39.0",
# ]
# ///
"""Convert stallguard_cli diagnostic logs into a synchronized Rerun (.rrd) replay.

Visualizes the 2D X-Z manipulator gantry schematic (cantilever post at X=0,
+Z up, soft-limit safe envelope, moving Z column + X carriage, motion trail,
and stall markers) synchronized on a single `controller_time` timeline alongside
50 Hz StallGuard4 traces, EMA baselines, 0.72x drop thresholds, position,
velocity, driver current/temperature, and CLI/event TextLogs.

Usage (via uv, no manual venv setup needed):
  uv run MotorControls\\MotionFirmware\\tools\\stallguard_to_rerun.py ^
      --log path\\to\\diag.log --out path\\to\\movement_replay.rrd --spawn
"""

from __future__ import annotations

import argparse
import re
import subprocess
from collections import deque
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb


def parse_log_timeline(log_path: Path) -> dict:
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    status_frames: list[dict] = []
    sg_frames: list[dict] = []
    events: list[dict] = []

    cur_t_s: float | None = None
    pending_status_t: float | None = None
    pending_axes: dict[str, dict] = {}

    pat_hdr = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\[\s*(\d+\.\d+)\]\s+(\w+),\s+mode\s+(\w+)"
    )
    pat_ax = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\s+([ZX])\s+(\S+)\s+pos[?\s]\s*([-+]?\d+\.\d+)\s+mm\s+v\s+([-+]?\d+\.\d+)\s+mm/s.*?SG\s+(\d+)\s+CS\s+(\d+)\s+([-+]?\d+\.\d+)C\s+cyc\s+(\d+)"
    )
    pat_sg = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?sg,([ZX]),(\d+),([-+]?\d+\.\d+),(\d+),(\d+),(\d+),(\d+)$"
    )
    pat_event = re.compile(
        r"^(?:\[\d+:\d+:\d+\.\d+\]\s+)?\[\s*(\d+\.\d+)\]\s+(\[(?:HOME|BOUNCE|SYS|ENC|JOG|FAULT)\].+)$"
    )
    pat_stall = re.compile(
        r"\[(HOME|BOUNCE)\]\s+([ZX]):\s+stall at (\w+) end,\s+([-+]?\d+\.\d+)\s+mm\s+\(([^,]+),\s+SG4\s+(\d+),\s+baseline\s+(\d+),\s+SG4_THRS\s+(\d+)\)"
    )
    pat_done = re.compile(
        r"\[HOME\]\s+([ZX]):\s+done - travel ([\d.]+) mm.*soft ([\d.]+)\.\.([\d.]+) mm"
    )
    pat_cmd = re.compile(r"^\[\d+:\d+:\d+\.\d+\]\s+>>>\s+(.+)$")

    def flush_pending():
        nonlocal pending_status_t, pending_axes
        if pending_status_t is not None and pending_axes:
            status_frames.append({"t_s": pending_status_t, "axes": dict(pending_axes)})
        pending_status_t = None
        pending_axes = {}

    for ln in lines:
        m = pat_hdr.match(ln)
        if m:
            flush_pending()
            cur_t_s = float(m.group(1))
            pending_status_t = cur_t_s
            continue
        m = pat_ax.match(ln)
        if m and pending_status_t is not None:
            pending_axes[m.group(1)] = {
                "state": m.group(2),
                "pos_mm": float(m.group(3)),
                "vel_mm_s": float(m.group(4)),
                "sg": int(m.group(5)),
                "cs": int(m.group(6)),
                "temp_c": float(m.group(7)),
                "cyc": int(m.group(8)),
            }
            if len(pending_axes) == 2:
                flush_pending()
            continue
        m = pat_sg.match(ln)
        if m:
            sg_frames.append(
                {
                    "axis": m.group(1),
                    "t_s": int(m.group(2)) / 1000.0,
                    "pos_mm": float(m.group(3)),
                    "sg4": int(m.group(4)),
                    "baseline": int(m.group(5)),
                    "sg_thr": int(m.group(6)),
                    "flag": int(m.group(7)),
                }
            )
            continue
        m = pat_event.match(ln)
        if m:
            t_ev = float(m.group(1))
            msg = m.group(2)
            ev = {"t_s": t_ev, "kind": "log", "msg": msg}
            ms = pat_stall.search(msg)
            if ms:
                ev.update(
                    {
                        "kind": "stall",
                        "axis": ms.group(2),
                        "end": ms.group(3),
                        "pos_mm": float(ms.group(4)),
                        "source": ms.group(5),
                        "sg4": int(ms.group(6)),
                        "baseline": int(ms.group(7)),
                        "sg4_thrs": int(ms.group(8)),
                    }
                )
            md = pat_done.search(msg)
            if md:
                ev.update(
                    {
                        "kind": "home_done",
                        "axis": md.group(1),
                        "travel_mm": float(md.group(2)),
                        "soft_min": float(md.group(3)),
                        "soft_max": float(md.group(4)),
                    }
                )
            events.append(ev)
            continue
        m = pat_cmd.match(ln)
        if m and cur_t_s is not None:
            events.append(
                {"t_s": cur_t_s, "kind": "cmd", "msg": f">>> {m.group(1).strip()}"}
            )

    flush_pending()
    return {"status": status_frames, "sg": sg_frames, "events": events}


def build_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(
                    origin="/gantry",
                    name="2D X-Z Gantry Schematic",
                    visual_bounds=rrb.VisualBounds2D(
                        x_range=[-30.0, 245.0], y_range=[-135.0, 22.0]
                    ),
                ),
                rrb.TextLogView(
                    origin="/logs",
                    name="CLI Commands & Homing Events",
                ),
                row_shares=[0.68, 0.32],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    origin="/plots/z/motion",
                    name="Z Axis - Position (mm) & Velocity (mm/s)",
                ),
                rrb.TimeSeriesView(
                    origin="/plots/z/sg4",
                    name="Z Axis - StallGuard4, Baseline & 0.72x Threshold",
                ),
                rrb.TimeSeriesView(
                    origin="/plots/x/motion",
                    name="X Axis - Position (mm) & Velocity (mm/s)",
                ),
                rrb.TimeSeriesView(
                    origin="/plots/x/sg4",
                    name="X Axis - StallGuard4, Baseline & 0.72x Threshold",
                ),
                rrb.TimeSeriesView(
                    origin="/plots/drivers",
                    name="TMC2240 Chip Temperature (C) & Current Scale (CS)",
                ),
                row_shares=[0.20, 0.23, 0.20, 0.23, 0.14],
            ),
            column_shares=[0.42, 0.58],
        ),
        collapse_panels=True,
    )


def export_rrd(log_path: Path, rrd_path: Path) -> dict:
    parsed = parse_log_timeline(log_path)
    status = parsed["status"]
    sg = parsed["sg"]
    events = parsed["events"]

    active_t = [
        f["t_s"]
        for f in status
        if any(ax_d["state"] != "IDLE" for ax_d in f["axes"].values())
    ]
    if not active_t:
        raise RuntimeError(f"No active motion frames found in {log_path}")
    t0 = min(active_t) - 1.0

    stream: list[tuple[float, int, str, dict]] = []
    for f in status:
        if f["t_s"] >= t0:
            stream.append((f["t_s"] - t0, 1, "status", f))
    for r in sg:
        if r["t_s"] >= t0:
            stream.append((r["t_s"] - t0, 0, "sg", r))
    for ev in events:
        if ev["t_s"] >= t0:
            stream.append((ev["t_s"] - t0, 2, "event", ev))
    stream.sort(key=lambda item: (item[0], item[1]))

    rrd_path.parent.mkdir(parents=True, exist_ok=True)
    rr.init("xz_manipulator_diagnostics", spawn=False)
    rr.save(str(rrd_path), default_blueprint=build_blueprint())

    # Static gantry structure & series styling
    rr.log(
        "/gantry/structure/cantilever_post",
        rr.Boxes2D(
            mins=[[-18.0, -125.0]],
            sizes=[[18.0, 130.0]],
            colors=[[90, 98, 112]],
            labels=["Cantilever Post (X=0 MIN)"],
        ),
        static=True,
    )
    rr.log(
        "/gantry/structure/x_rail",
        rr.LineStrips2D(
            [[[0.0, 4.0], [228.0, 4.0]]],
            colors=[[120, 128, 140]],
            radii=[0.8],
        ),
        static=True,
    )
    rr.log(
        "/gantry/structure/z_bounds",
        rr.LineStrips2D(
            [[[0.0, 0.0], [228.0, 0.0]], [[0.0, -116.0], [228.0, -116.0]]],
            colors=[[190, 90, 90], [190, 90, 90]],
            radii=[0.4, 0.4],
        ),
        static=True,
    )

    rr.log(
        "/plots/z/motion/pos_mm",
        rr.SeriesLines(colors=[34, 108, 205], names="Z pos (mm)", widths=2.0),
        static=True,
    )
    rr.log(
        "/plots/z/motion/vel_mm_s",
        rr.SeriesLines(colors=[230, 115, 30], names="Z vel (mm/s)", widths=1.5),
        static=True,
    )
    rr.log(
        "/plots/x/motion/pos_mm",
        rr.SeriesLines(colors=[34, 108, 205], names="X pos (mm)", widths=2.0),
        static=True,
    )
    rr.log(
        "/plots/x/motion/vel_mm_s",
        rr.SeriesLines(colors=[230, 115, 30], names="X vel (mm/s)", widths=1.5),
        static=True,
    )

    for ax in ("z", "x"):
        rr.log(
            f"/plots/{ax}/sg4/sg4_50hz",
            rr.SeriesLines(
                colors=[42, 155, 75], names=f"{ax.upper()} SG4 (50Hz)", widths=2.0
            ),
            static=True,
        )
        rr.log(
            f"/plots/{ax}/sg4/sg4_4hz",
            rr.SeriesLines(
                colors=[155, 165, 175], names=f"{ax.upper()} SG4 (4Hz)", widths=1.2
            ),
            static=True,
        )
        rr.log(
            f"/plots/{ax}/sg4/baseline",
            rr.SeriesLines(
                colors=[140, 70, 175], names=f"{ax.upper()} Baseline", widths=2.0
            ),
            static=True,
        )
        rr.log(
            f"/plots/{ax}/sg4/drop_thresh_0_72x",
            rr.SeriesLines(
                colors=[215, 55, 55], names=f"{ax.upper()} 0.72x Threshold", widths=1.5
            ),
            static=True,
        )

    rr.log(
        "/plots/drivers/z_temp_c",
        rr.SeriesLines(colors=[210, 80, 50], names="Z TMC2240 Temp (C)"),
        static=True,
    )
    rr.log(
        "/plots/drivers/x_temp_c",
        rr.SeriesLines(colors=[50, 140, 210], names="X TMC2240 Temp (C)"),
        static=True,
    )
    rr.log(
        "/plots/drivers/z_cs",
        rr.SeriesLines(colors=[180, 130, 40], names="Z CS (0..31)"),
        static=True,
    )
    rr.log(
        "/plots/drivers/x_cs",
        rr.SeriesLines(colors=[90, 160, 130], names="X CS (0..31)"),
        static=True,
    )

    cur_pos = {"Z": 0.0, "X": 0.0}
    cur_state = {"Z": "IDLE", "X": "IDLE"}
    soft_bounds = {"Z": (5.0, 110.7), "X": (10.0, 214.1)}
    travel_est = {"Z": 115.7, "X": 224.1}
    trail: deque[list[float]] = deque(maxlen=180)
    last_stall_pos: tuple[float, float] | None = None
    last_stall_t: float = -10.0
    stall_visible = False
    last_gantry_t: float = -1.0

    def update_gantry(t_rel: float, force: bool = False) -> None:
        nonlocal last_gantry_t, stall_visible
        if not force and (t_rel - last_gantry_t) < 0.035:
            return
        last_gantry_t = t_rel
        xp = cur_pos["X"]
        zp = cur_pos["Z"]
        y_screen = -zp
        trail.append([xp, y_screen])

        z_lo, z_hi = soft_bounds["Z"]
        x_lo, x_hi = soft_bounds["X"]
        rr.log(
            "/gantry/soft_envelope",
            rr.Boxes2D(
                mins=[[x_lo, -z_hi]],
                sizes=[[x_hi - x_lo, z_hi - z_lo]],
                colors=[[70, 160, 95]],
                labels=[
                    f"Soft Zone X:{x_lo:.1f}..{x_hi:.1f}, Z:{z_lo:.1f}..{z_hi:.1f}"
                ],
            ),
        )
        rr.log(
            "/gantry/x_carriage",
            rr.LineStrips2D(
                [[[xp, 4.0], [xp, -(travel_est["Z"] + 2.0)]]],
                colors=[[40, 115, 220]],
                radii=[0.9],
            ),
        )
        rr.log(
            "/gantry/end_effector",
            rr.Boxes2D(
                centers=[[xp, y_screen]],
                sizes=[[10.0, 8.0]],
                colors=[[30, 115, 230]],
                labels=[
                    f"X={xp:.2f} ({cur_state['X']}) | Z={zp:.2f} ({cur_state['Z']})"
                ],
            ),
        )
        if len(trail) >= 2:
            rr.log(
                "/gantry/trail",
                rr.LineStrips2D([list(trail)], colors=[[95, 165, 240]], radii=[0.45]),
            )
        if last_stall_pos is not None and (t_rel - last_stall_t) <= 1.2:
            rr.log(
                "/gantry/stall_flash",
                rr.Points2D(
                    [[last_stall_pos[0], -last_stall_pos[1]]],
                    colors=[[225, 45, 45]],
                    radii=[4.5],
                    labels=["STALL TRIP"],
                ),
            )
            stall_visible = True
        elif stall_visible:
            rr.log("/gantry/stall_flash", rr.Clear(recursive=False))
            stall_visible = False

    for t_rel, _prio, kind, payload in stream:
        rr.set_time("controller_time", duration=t_rel)
        if kind == "sg":
            ax_u = payload["axis"]
            ax_l = ax_u.lower()
            cur_pos[ax_u] = payload["pos_mm"]
            rr.log(f"/plots/{ax_l}/motion/pos_mm", rr.Scalars(payload["pos_mm"]))
            rr.log(f"/plots/{ax_l}/sg4/sg4_50hz", rr.Scalars(float(payload["sg4"])))
            if payload["baseline"] > 0:
                rr.log(
                    f"/plots/{ax_l}/sg4/baseline",
                    rr.Scalars(float(payload["baseline"])),
                )
                rr.log(
                    f"/plots/{ax_l}/sg4/drop_thresh_0_72x",
                    rr.Scalars(0.72 * float(payload["baseline"])),
                )
            update_gantry(t_rel)
        elif kind == "status":
            for ax_u, ad in payload["axes"].items():
                ax_l = ax_u.lower()
                cur_pos[ax_u] = ad["pos_mm"]
                cur_state[ax_u] = ad["state"]
                rr.log(f"/plots/{ax_l}/motion/pos_mm", rr.Scalars(ad["pos_mm"]))
                rr.log(f"/plots/{ax_l}/motion/vel_mm_s", rr.Scalars(ad["vel_mm_s"]))
                rr.log(f"/plots/{ax_l}/sg4/sg4_4hz", rr.Scalars(float(ad["sg"])))
                rr.log(f"/plots/drivers/{ax_l}_temp_c", rr.Scalars(ad["temp_c"]))
                rr.log(f"/plots/drivers/{ax_l}_cs", rr.Scalars(float(ad["cs"])))
            update_gantry(t_rel, force=True)
        elif kind == "event":
            ev_kind = payload["kind"]
            level = rr.TextLogLevel.WARN if ev_kind == "stall" else rr.TextLogLevel.INFO
            rr.log("/logs", rr.TextLog(payload["msg"], level=level))
            if ev_kind == "stall":
                ax_u = payload["axis"]
                ax_l = ax_u.lower()
                cur_pos[ax_u] = payload["pos_mm"]
                last_stall_pos = (cur_pos["X"], cur_pos["Z"])
                last_stall_t = t_rel
                rr.log(
                    f"/plots/{ax_l}/sg4/sg4_50hz",
                    rr.Scalars(float(payload["sg4"])),
                )
                update_gantry(t_rel, force=True)
            elif ev_kind == "home_done":
                ax_u = payload["axis"]
                soft_bounds[ax_u] = (payload["soft_min"], payload["soft_max"])
                travel_est[ax_u] = payload["travel_mm"]

    rr.disconnect()
    return {
        "status_frames": len(status),
        "sg_frames": len(sg),
        "events": len(events),
        "duration_s": round(stream[-1][0], 2) if stream else 0.0,
    }


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Export stallguard_cli log to Rerun (.rrd)"
    )
    ap.add_argument("--log", type=Path, required=True, help="Input diag_*.log path")
    ap.add_argument("--out", type=Path, required=True, help="Output .rrd path")
    ap.add_argument(
        "--spawn", action="store_true", help="Open Rerun viewer with the .rrd"
    )
    args = ap.parse_args()

    stats = export_rrd(args.log, args.out)
    print(f"SAVED {args.out} ({stats})")
    if args.spawn:
        subprocess.Popen(["rerun", str(args.out)])


if __name__ == "__main__":
    main()
