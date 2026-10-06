import importlib.util
import json
import math
import random
from dataclasses import asdict, replace
from pathlib import Path

import pytest
from motion_diagnostics.commands import (
    Command,
    control_packet,
    decode_command,
    encode_detection_packet,
)
from motion_diagnostics.correlation import Correlator
from motion_diagnostics.crc import crc16_ccitt
from motion_diagnostics.csv_io import CsvWriter, derive, read_csv, record_rows
from motion_diagnostics.frames import (
    CommandResult,
    Header,
    HomeAxis,
    HomeResult,
    Phase,
    Status,
    StatusPayload,
    StreamDecoder,
    decode_record,
    encode_record,
)
from motion_diagnostics.metrics import analyze, summarize
from motion_diagnostics.status_bits import StatusBits, command_succeeded, host_ready

ROOT = Path(__file__).resolve().parents[3]
CONFIG = {"qualified": True, "config_id": 1}
RECORDS = [
    CommandResult(
        Header(record_seq=1, command_seq=1, status_bits=1, phase_count=3),
        (
            Phase(
                axis_flags=255, unwrap_start=-8192, unwrap_end=8192, unwrap_delta=16384
            ),
            Phase(axis=1, phase=2),
            Phase(axis=1, phase=3, emitted_delta_steps=-100),
        ),
    ),
    HomeResult(
        axes=(
            HomeAxis(home_offset_counts=-1234),
            HomeAxis(axis=1, seek_emitted_steps=-55),
        )
    ),
    Status(status=StatusPayload(x_position_steps=-42, z_unwrap_counts=8192)),
]


@pytest.mark.parametrize("record", RECORDS)
def test_records_roundtrip(record):
    wire = encode_record(record)
    assert len(wire) == record.header.payload_len + 44
    assert decode_record(wire) == record
    assert encode_record(decode_record(wire)) == wire


@pytest.mark.parametrize(
    "record,boundary",
    [(r, b) for r in RECORDS for b in range(r.header.payload_len + 45)],
)
def test_every_chunk_boundary(record, boundary):
    wire = encode_record(record)
    decoder = StreamDecoder()
    assert decoder.feed(wire[:boundary]) + decoder.feed(wire[boundary:]) == [record]
    assert not decoder.buffer


@pytest.mark.parametrize("offset", [2, 3, 4, 5, 6, 39, 40, 200, 352, 354, 355])
def test_corruption_resync(offset):
    frame = bytearray(encode_record(RECORDS[0]))
    frame[offset] ^= 0xFF
    decoder = StreamDecoder()
    good = encode_record(RECORDS[0])
    assert decoder.feed(b"noise\xd3" + bytes(frame) + good) == [RECORDS[0]]
    assert decoder.errors
    assert len(decoder.buffer) <= 355


def test_preamble_tail_payload_and_bytewise():
    p = Phase(unwrap_start=int.from_bytes(b"\xd3\x7e\r\n", "little"))
    record = CommandResult(phases=(p, Phase(axis=1, phase=2), Phase(axis=1, phase=3)))
    wire = encode_record(record)
    decoder = StreamDecoder()
    result = []
    for byte in wire * 2:
        result += decoder.feed(bytes([byte]))
    assert result == [record, record]


def test_crc_golden_commands_and_corruption():
    assert crc16_ccitt(b"123456789") == 0x29B1
    wire = encode_detection_packet(1, 1, 150, 100, 0, 0)
    assert wire.hex().upper() == "AA5501010001DC05E803000000009A740D0A"
    assert decode_command(wire).x_01mm == 1500
    for kind in (2, 3, 4):
        assert decode_command(control_packet(kind, 55)).command_type == kind
    bad = bytearray(wire)
    bad[8] ^= 1
    with pytest.raises(ValueError):
        decode_command(bytes(bad))


def test_deployed_encoder_randomized_equality():
    spec = importlib.util.spec_from_file_location(
        "deployed_protocol",
        ROOT / "VisionSystem" / "04_Source_Code" / "utils" / "uart_protocol.py",
    )
    deployed = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(deployed)
    rng = random.Random(52)
    for _ in range(500):
        args = (
            rng.randrange(65536),
            rng.randrange(3),
            rng.uniform(-100, 300),
            rng.uniform(-100, 300),
            rng.randrange(3600) / 10,
            rng.randrange(-1800, 1801) / 10,
        )
        assert encode_detection_packet(*args) == deployed.encode_detection_packet(*args)
    assert control_packet(2, 27) == deployed.encode_heartbeat_packet(27)
    assert (
        decode_command(encode_detection_packet(1, 1, 0, 0, 359.99, 0)).angle_01deg == 0
    )


def test_metric_known_vectors_and_no_data():
    stats = summarize([-1, 0, 1], [0, 1, 2])
    assert stats["bias"] == 0
    assert stats["max_abs_error"] == stats["sample_sd"] == 1
    assert stats["RP_1D_adaptation"] == pytest.approx(2 / 3 + 3 / math.sqrt(3))
    for values in ([], [3]):
        result = summarize(values)
        assert result["sample_sd"] is None and result["RP_1D_adaptation"] is None
    assert summarize([])["bias"] is None
    assert summarize([720, 1080])["bias"] == 900


def test_derived_delta_endpoint_and_exclusion():
    p = Phase(
        axis_flags=255,
        as_status_start=0x20,
        as_status_end=0x20,
        mscnt_check=1,
        start_position_steps=3200,
        applied_target_01mm=800,
        requested_target_01mm=800,
        commanded_delta_steps=3200,
        emitted_delta_steps=3200,
        unwrap_start=4196,
        unwrap_end=8292,
        home_offset_counts=0,
    )
    result = derive(p, Header(status_bits=1), CONFIG)
    assert result["total_error_um"] == 0
    assert result["endpoint_error_um"] == pytest.approx(100 * 40000 / 4096)
    assert result["step_implied_deg"] == 360
    assert result["metrics_eligible"]
    assert not derive(
        replace(p, axis_flags=p.axis_flags | 256), Header(status_bits=1), CONFIG
    )["metrics_eligible"]
    assert not derive(p, Header(status_bits=1))["metrics_eligible"]
    assert "encoder_um" not in derive(Phase(), Header(), CONFIG)
    negative = derive(
        replace(p, axis_flags=p.axis_flags | 4096, unwrap_end=100),
        Header(status_bits=1),
        CONFIG,
    )
    assert negative["encoder_um"] == 40000


def test_grouping_and_unwrapped_angles():
    rows = []
    for direction in ("positive", "negative"):
        for err in (-1, 0, 1):
            rows.append(
                {
                    "record_type": 129,
                    "axis": 0,
                    "phase": 1,
                    "applied_target_01mm": 10,
                    "approach_direction": direction,
                    "metrics_eligible": True,
                    "endpoint_error_um": err,
                    "shaft_error_deg": 720 + err,
                    "total_error_um": err,
                    "shaft_tracking_error_um": err,
                }
            )
    rows.append(
        {
            **rows[0],
            "metrics_eligible": False,
            "exclusion_reason": "clamped",
            "endpoint_error_um": 999,
        }
    )
    groups = analyze(rows)
    assert len(groups) == 2
    assert sum(g["excluded"] for g in groups) == 1
    assert all(g["endpoint"]["sample_sd"] == 1 for g in groups)
    assert all(g["angular_unwrapped"]["bias"] == 720 for g in groups)
    assert analyze([]) == []


def reply(command, ordinal=1, record_seq=1, timestamp=100):
    return CommandResult(
        Header(
            command_seq=ordinal,
            record_seq=record_seq,
            timestamp_ms=timestamp,
            obj_id=command.obj_id,
            class_id=command.class_id,
            command_type=command.command_type,
            rx_x_01mm=command.x_01mm,
            rx_y_01mm=command.y_01mm,
            rx_angle_01deg=command.angle_01deg,
            rx_corr_01deg=command.corr_01deg,
        )
    )


def test_correlation_missing_late_duplicate_payload_and_reset():
    c = Correlator("session", timeout=1)
    cmd = Command(obj_id=12, x_01mm=123)
    sent = c.register(cmd)
    with pytest.raises(ValueError):
        c.register(cmd)
    c.mark_sent(sent.host_send_seq)
    wrong = reply(replace(cmd, x_01mm=99))
    assert c.match(wrong)["match_status"] == "unmatched"
    assert c.expire(sent.registered_at + 2) == [sent]
    matched = c.match(reply(cmd))
    assert matched["match_status"] == "late"
    assert matched["host_send_seq"] == sent.host_send_seq
    assert c.match(reply(cmd))["match_status"] == "duplicate"
    assert c.counts["missing"] == c.counts["late"] == c.counts["duplicate"] == 1
    new_sent = c.register(cmd)
    c.mark_sent(new_sent.host_send_seq)
    assert c.match(reply(cmd, 2, 2, 101))["host_send_seq"] == 2
    old_session = c.session_id
    assert c.match(reply(cmd, 1, 0, 0))["match_status"] == "unmatched"
    assert c.session_id != old_session


def test_csv_roundtrip(tmp_path):
    path = tmp_path / "result.csv"
    # pytest's base directory is explicitly configured inside the project.
    with CsvWriter(path) as writer:
        for record in RECORDS:
            writer.write(
                record_rows(record, {"session_id": "abc", "host_send_seq": 3}, CONFIG)
            )
    rows = read_csv(path)
    assert len(rows) == 6
    assert rows[0]["session_id"] == "abc"
    assert rows[0]["unwrap_start"] == "-8192"
    assert rows[-1]["endpoint_um"] == ""
    assert rows[3]["home_home_offset_counts"] == "-1234"


def test_replayed_old_ordinal_cannot_match_reused_id():
    c = Correlator("original")
    cmd = Command(obj_id=7)
    first = c.register(cmd)
    c.mark_sent(first.host_send_seq)
    assert c.match(reply(cmd, 10, 1, 1))["match_status"] == "matched"
    second = c.register(cmd)
    c.mark_sent(second.host_send_seq)
    # Even if bounded duplicate history has expired, the ordinal anchor remains.
    c.seen.clear()
    assert c.match(reply(cmd, 10, 2, 2))["match_status"] == "unmatched"
    assert c.match(reply(cmd, 11, 3, 3))["host_send_seq"] == second.host_send_seq
    c.reset()
    assert not c.mark_sent(second.host_send_seq)


def test_pending_send_is_not_successful_transmission():
    c = Correlator()
    cmd = Command(obj_id=17)
    sent = c.register(cmd)
    assert c.match(reply(cmd))["match_status"] == "unmatched"
    assert not sent.completed
    c.mark_sent(sent.host_send_seq, False)
    assert c.match(reply(cmd))["match_status"] == "unmatched"


def test_cross_language_fixtures():
    path = (
        ROOT
        / "MotorControls"
        / "MotionFirmware"
        / "Tests"
        / "fixtures"
        / "diag_records_v1.json"
    )
    if not path.exists():
        pytest.skip("firmware C1 diag_records_v1.json not generated yet")
    fixtures = json.loads(path.read_text(encoding="utf-8"))
    assert fixtures, "fixture array must be nonempty"
    for fixture in fixtures:
        wire = bytes.fromhex(fixture["hex"])
        record = decode_record(wire)
        assert encode_record(record) == wire
        assert record.header.record_type == fixture["type"], fixture["name"]
        actual = asdict(record)
        actual.update(actual["header"])
        if isinstance(record, HomeResult):
            actual["home"] = actual["axes"]

        def check(expected, value, path="fields", fixture=fixture):
            if isinstance(expected, dict):
                for key, item in expected.items():
                    assert key in value, (
                        f"{fixture['name']}: unknown fixture field {path}.{key}"
                    )
                    check(item, value[key], f"{path}.{key}")
            elif isinstance(expected, list):
                assert len(value) == len(expected), (
                    f"{fixture['name']}: {path} length: "
                    f"fixture={len(expected)}, decoded={len(value)}"
                )
                for index, (a, b) in enumerate(zip(expected, value)):
                    check(a, b, f"{path}[{index}]")
            else:
                assert value == expected, (
                    f"{fixture['name']}: {path}: fixture={expected!r}, decoded={value!r}"
                )

        check(fixture["fields"], actual)


def test_timestamp_wrap_and_delayed_replay():
    c = Correlator("wrap")
    cmd = Command(obj_id=1)
    first = c.register(cmd)
    c.mark_sent(first.host_send_seq)
    old_record = reply(cmd, 1, 1, 0xFFFFFFFE)
    assert c.match(old_record)["timestamp_ms_extended"] == 0xFFFFFFFE
    cmd2 = replace(cmd, obj_id=2)
    sent2 = c.register(cmd2)
    c.mark_sent(sent2.host_send_seq)
    assert c.match(reply(cmd2, 2, 2, 3))["timestamp_ms_extended"] == (1 << 32) + 3
    duplicate = c.match(old_record)
    assert duplicate["match_status"] == "duplicate"
    assert duplicate["timestamp_ms_extended"] == 0xFFFFFFFE
    assert c.session_id == "wrap"


def test_encoder_metrics_independent_of_mscnt_qualification():
    p = Phase(
        axis_flags=0x3F,
        as_status_start=0x20,
        as_status_end=0x20,
        mscnt_check=3,
        driver_mode=0x14,
        unwrap_end=4096,
        requested_target_01mm=400,
        applied_target_01mm=400,
        commanded_delta_steps=3200,
        emitted_delta_steps=3200,
    )
    h = Header(status_bits=0x801 | StatusBits.RX_OVERFLOW)
    result = derive(p, h, CONFIG)
    assert result["metrics_eligible"]
    assert result["endpoint_error_um"] == 0
    rows = record_rows(
        CommandResult(h, (p, Phase(axis=1, phase=2), Phase(axis=1, phase=3))),
        {"match_status": "matched"},
        CONFIG,
    )
    assert rows[0]["mscnt_check"] == 3 and rows[0]["driver_mode"] == 0x14
    assert analyze(rows)[0]["endpoint"]["n"] == 1
    assert not derive(
        p, replace(h, status_bits=h.status_bits | StatusBits.DRIVER_FAULT), CONFIG
    )["metrics_eligible"]
    assert not derive(
        p, replace(h, status_bits=h.status_bits | StatusBits.CLAMPED), CONFIG
    )["metrics_eligible"]


@pytest.mark.parametrize(
    "bit",
    [
        StatusBits.MSCNT_UNQUALIFIED,
        StatusBits.RX_OVERFLOW,
        StatusBits.CANCELLED,
        StatusBits.REJECTED,
        StatusBits.CLAMPED,
    ],
)
def test_event_bits_do_not_change_readiness_or_success(bit):
    assert host_ready(3, bit)
    assert command_succeeded(StatusBits.SUCCESS | bit)
    assert not command_succeeded(bit)
    assert not host_ready(1, bit)


@pytest.mark.parametrize(
    "bit",
    [
        StatusBits.ABORTED,
        StatusBits.NOT_HOMED,
        StatusBits.DRIVER_FAULT,
        StatusBits.ENCODER_INVALID,
        StatusBits.SAMPLE_OVERRUN,
        StatusBits.STEP_TIMING_FAULT,
        StatusBits.MSCNT_MISMATCH,
        StatusBits.TX_BACKPRESSURE,
        StatusBits.HOME_FAILED,
        StatusBits.LIMIT_FAULT,
        StatusBits.CONFIG_INVALID,
        StatusBits.POSITION_UNCERTAIN,
    ],
)
def test_fault_bits_inhibit_readiness_and_success(bit):
    assert not host_ready(3, StatusBits.MSCNT_UNQUALIFIED | bit)
    assert not command_succeeded(StatusBits.SUCCESS | bit)


def test_fault_mask_matches_design_contract():
    from motion_diagnostics.status_bits import FAULT_BITS

    assert FAULT_BITS == 0x0003E7E2


@pytest.mark.parametrize("kind", [2, 3, 4])
def test_control_timeout_releases_id_but_pick_timeout_does_not(kind):
    c = Correlator(timeout=1)
    cmd = Command(command_type=kind, obj_id=7, class_id=0)
    sent = c.register(cmd)
    c.mark_sent(sent.host_send_seq)
    c.expire(sent.registered_at + 2)
    assert sent.state == "expired"
    assert c.register(cmd).host_send_seq == 2
    pick = c.register(Command(obj_id=8))
    c.mark_sent(pick.host_send_seq)
    c.expire(pick.registered_at + 2)
    with pytest.raises(ValueError, match="already outstanding"):
        c.register(pick.command)


def test_rehome_rejected_status_matches_exact_echo():
    c = Correlator()
    cmd = Command(command_type=4, obj_id=77, class_id=0)
    sent = c.register(cmd)
    c.mark_sent(sent.host_send_seq)
    h = Header(
        record_type=0x83,
        payload_len=72,
        command_seq=9,
        record_seq=1,
        command_type=4,
        obj_id=77,
        status_bits=StatusBits.REJECTED | StatusBits.RX_OVERFLOW,
    )
    record = Status(h, StatusPayload(state=3, homed_mask=3))
    matched = c.match(record)
    assert matched["match_status"] == "rejected"
    assert matched["host_send_seq"] == sent.host_send_seq
    assert sent.completed
    assert c.counts["missing"] == 0
