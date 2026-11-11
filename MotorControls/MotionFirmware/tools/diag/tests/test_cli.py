import json
import time
from dataclasses import replace

import pytest
from diag.cli import main
from motion_diagnostics.commands import decode_command
from motion_diagnostics.csv_io import read_csv
from motion_diagnostics.frames import (
    CommandResult,
    Header,
    HomeResult,
    Phase,
    Status,
    StatusPayload,
    encode_record,
)


class FirmwareLoopback:
    def __init__(self, port):
        self.rx = bytearray()
        self.commands = []
        self.ordinal = 0
        self.closed = False
        self.drop_pick = "timeout" in port
        self.position = 0
        self.reject_home = "reject" in port
        self.fault_pick = "fault" in port

    def write(self, frame):
        command = decode_command(frame)
        self.commands.append(command)
        self.ordinal += 1
        h = Header(
            record_seq=self.ordinal,
            command_seq=self.ordinal,
            timestamp_ms=self.ordinal,
            obj_id=command.obj_id,
            class_id=command.class_id,
            command_type=command.command_type,
            rx_x_01mm=command.x_01mm,
            rx_y_01mm=command.y_01mm,
            rx_angle_01deg=command.angle_01deg,
            rx_corr_01deg=command.corr_01deg,
            status_bits=0x801 if command.command_type in (1, 4) else 0x1800,
            home_epoch=1,
        )
        if command.command_type == 1:
            if self.drop_pick:
                return len(frame)
            target = command.x_01mm * 8
            if self.fault_pick:
                h = replace(h, status_bits=h.status_bits | 0x40)
            phase = Phase(
                axis_flags=63,
                as_status_start=0x20,
                as_status_end=0x20,
                requested_target_01mm=command.x_01mm,
                applied_target_01mm=command.x_01mm,
                start_position_steps=self.position,
                target_position_steps=target,
                commanded_delta_steps=target - self.position,
                emitted_delta_steps=target - self.position,
                mscnt_check=3,
                unwrap_start=round(self.position * 4096 / 3200),
                unwrap_end=round(target * 4096 / 3200),
            )
            result = CommandResult(
                replace(h, phase_count=3),
                (
                    phase,
                    Phase(
                        axis=1,
                        phase=2,
                        axis_flags=63,
                        mscnt_check=3,
                        as_status_start=0x20,
                        as_status_end=0x20,
                    ),
                    Phase(
                        axis=1,
                        phase=3,
                        axis_flags=63,
                        mscnt_check=3,
                        as_status_start=0x20,
                        as_status_end=0x20,
                    ),
                ),
            )
            self.position = target
        elif command.command_type == 4:
            result = (
                Status(
                    replace(h, record_type=0x83, payload_len=72, status_bits=0x1808),
                    StatusPayload(state=3, homed_mask=3),
                )
                if self.reject_home
                else HomeResult(replace(h, record_type=0x82, payload_len=80))
            )
        else:
            if command.command_type == 3:
                h = replace(h, status_bits=0x20002)
            result = Status(
                replace(h, record_type=0x83, payload_len=72),
                StatusPayload(state=4, homed_mask=0)
                if command.command_type == 3
                else StatusPayload(state=2, homed_mask=3),
            )
        self.rx.extend(encode_record(result))
        return len(frame)

    def read(self, size):
        chunk = bytes(self.rx[:17])
        del self.rx[:17]
        if not chunk:
            time.sleep(0.001)
        return chunk

    def close(self):
        self.closed = True


@pytest.fixture
def setup(tmp_path):
    config = tmp_path / "config.json"
    config.write_text(
        json.dumps(
            {
                "qualified": True,
                "config_id": 1,
                "usable_x_mm": 300,
                "safe_z_01mm": 0,
                "pick_depth_01mm": 0,
            }
        ),
        encoding="utf-8",
    )
    instances = []

    def factory(port):
        obj = FirmwareLoopback(port)
        instances.append(obj)
        return obj

    return config, tmp_path / "run.csv", factory, instances


@pytest.mark.parametrize("action", ["heartbeat", "abort", "rehome"])
def test_controls(action, setup):
    config, output, factory, instances = setup
    assert (
        main(
            [action, "--port", "fake", "--csv", str(output), "--config", str(config)],
            factory,
        )
        == 0
    )
    assert len(read_csv(output)) == (2 if action == "rehome" else 1)
    assert instances[0].closed


def test_sequence_and_analyze(setup):
    config, output, factory, instances = setup
    args = [
        "send-sequence",
        "--port",
        "fake",
        "--csv",
        str(output),
        "--config",
        str(config),
        "--targets-mm",
        "25,100",
        "--approach-mm",
        "5",
        "--reverse-approach-mm",
        "200",
        "--direction",
        "both",
        "--repetitions",
        "2",
        "--pacing",
        "0",
    ]
    assert main(args, factory) == 0
    rows = read_csv(output)
    assert len(rows) == 1 + 16 * 3
    assert len([r for r in rows if r["scored"] == "True"]) == 8 * 3
    assert [c.x_01mm for c in instances[0].commands[1:5]] == [50, 250, 2000, 250]
    assert main(["analyze", "--csv", str(output)]) == 0
    report = json.loads(output.with_name("run_report.json").read_text(encoding="utf-8"))
    assert report["groups"]
    assert report["transport_quality"]["missing"] == 0
    assert any(g["endpoint"]["n"] > 0 for g in report["groups"])
    assert all(r["mscnt_check"] == "3" for r in rows if r["record_type"] == "129")
    assert all(g["endpoint"]["n"] == 2 for g in report["groups"] if g["eligible"])
    assert output.with_suffix(".bin").stat().st_size > 0


def test_sequence_timeout_aborts_without_retry(setup):
    config, output, factory, instances = setup
    assert (
        main(
            [
                "send-sequence",
                "--port",
                "timeout",
                "--csv",
                str(output),
                "--config",
                str(config),
                "--targets-mm",
                "25",
                "--approach-mm",
                "5",
                "--repetitions",
                "1",
                "--timeout",
                "0.2",
            ],
            factory,
        )
        == 1
    )
    assert [c.command_type for c in instances[0].commands] == [2, 1, 3]
    health = json.loads(output.with_suffix(".health.json").read_text(encoding="utf-8"))
    assert health["missing"] >= 1


def test_capture(setup):
    config, output, factory, instances = setup
    assert (
        main(
            [
                "capture",
                "--port",
                "fake",
                "--csv",
                str(output),
                "--config",
                str(config),
                "--duration",
                "0.01",
            ],
            factory,
        )
        == 0
    )
    assert instances[0].closed


def test_sequence_rejects_wrong_approach(setup):
    config, output, factory, instances = setup
    assert (
        main(
            [
                "send-sequence",
                "--port",
                "fake",
                "--csv",
                str(output),
                "--config",
                str(config),
                "--targets-mm",
                "25",
                "--approach-mm",
                "30",
                "--repetitions",
                "1",
            ],
            factory,
        )
        == 1
    )
    assert not instances[0].commands


def test_help():
    with pytest.raises(SystemExit) as exc:
        main(["--help"])
    assert exc.value.code == 0


def test_rehome_rejected_status_is_terminal_not_uncertain(setup, capsys):
    config, output, factory, instances = setup
    assert (
        main(
            [
                "rehome",
                "--port",
                "reject",
                "--csv",
                str(output),
                "--config",
                str(config),
            ],
            factory,
        )
        == 1
    )
    assert [c.command_type for c in instances[0].commands] == [4]
    text = capsys.readouterr().out
    assert "rehome rejected" in text
    assert "uncertain" not in text
    assert read_csv(output)[0]["match_status"] == "rejected"
    assert (
        json.loads(output.with_suffix(".health.json").read_text(encoding="utf-8"))[
            "missing"
        ]
        == 0
    )


def test_fault_result_stops_sequence(setup):
    config, output, factory, instances = setup
    assert (
        main(
            [
                "send-sequence",
                "--port",
                "fault",
                "--csv",
                str(output),
                "--config",
                str(config),
                "--targets-mm",
                "25",
                "--approach-mm",
                "5",
                "--repetitions",
                "2",
            ],
            factory,
        )
        == 1
    )
    assert [c.command_type for c in instances[0].commands] == [2, 1]
    assert all(
        r["metrics_eligible"] == "False"
        for r in read_csv(output)
        if r["record_type"] == "129"
    )
