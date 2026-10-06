import threading
import time
from dataclasses import replace

import pytest
from motion_diagnostics.commands import decode_command
from motion_diagnostics.csv_io import read_csv
from motion_diagnostics.frames import (
    CommandResult,
    Header,
    HomeAxis,
    HomeResult,
    Status,
    StatusPayload,
    encode_record,
)
from utils.angle_calculator import evaluasi_qc_dan_servo
from utils.diag_receiver import DiagnosticReceiver
from utils.uart_handler import KoneksiUART
from utils.uart_protocol import encode_detection_packet
from utils.vision_command import encode_runtime_detection_packet


class FakeSerial:
    def __init__(self):
        self.rx = bytearray()
        self.lock = threading.Lock()
        self.frames = []
        self.ordinal = 0
        self.is_open = True
        self.delay = 0
        self.short = False
        self.drop_heartbeats = 0
        self.continuous_noise = False
        self.reject_pick_ids = set()

    def write(self, frame):
        command = decode_command(frame)
        self.frames.append(frame)
        self.ordinal += 1
        if command.command_type == 2 and self.drop_heartbeats:
            self.drop_heartbeats -= 1
            return len(frame)
        h = Header(
            record_seq=self.ordinal,
            command_seq=self.ordinal,
            timestamp_ms=self.ordinal,
            status_bits=0x801 if command.command_type == 1 else 0x1800,
            obj_id=command.obj_id,
            class_id=command.class_id,
            command_type=command.command_type,
            rx_x_01mm=command.x_01mm,
            rx_y_01mm=command.y_01mm,
            rx_angle_01deg=command.angle_01deg,
            rx_corr_01deg=command.corr_01deg,
        )
        record = (
            CommandResult(
                replace(h, status_bits=0x8008)
                if command.obj_id in self.reject_pick_ids
                else h
            )
            if command.command_type == 1
            else Status(
                replace(h, record_type=0x83, payload_len=72),
                StatusPayload(state=2, homed_mask=3),
            )
        )
        with self.lock:
            self.rx.extend(encode_record(record))
        time.sleep(self.delay)
        return 1 if self.short else len(frame)

    def read(self, size):
        with self.lock:
            result = bytes(self.rx[:7])
            del self.rx[:7]
        if not result:
            time.sleep(0.002)
            if self.continuous_noise:
                return b"\x00"
        return result

    def close(self):
        self.is_open = False


def await_condition(condition, timeout=2):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() >= deadline:
            raise AssertionError("background condition timed out")
        time.sleep(0.005)


def test_raw_transport_racing_reply_and_shutdown(tmp_path, monkeypatch):
    ser = FakeSerial()
    ser.delay = 0.1
    monkeypatch.setattr(KoneksiUART, "_connect", lambda self: None)
    uart = KoneksiUART(csv_path=tmp_path / "live.csv")
    uart.ser = ser
    uart.receiver = DiagnosticReceiver(ser, uart.csv_path).start()
    await_condition(lambda: uart.receiver.ready)
    packet = encode_detection_packet(1, 1, 150, 100, 0, 0)
    started = time.monotonic()
    assert uart.kirim(packet, force=True)
    assert time.monotonic() - started < 0.05
    await_condition(lambda: len(read_csv(uart.csv_path)) == 4)
    receiver = uart.receiver
    rows = read_csv(uart.csv_path)
    assert receiver.ready
    assert rows[1]["status_bits"] == str(0x801)
    assert rows[1]["host_send_seq"] == "2" and rows[1]["match_status"] == "matched"
    assert rows[1]["obj_id"] == "1"
    assert ser.frames[-1] == packet
    with pytest.raises(TypeError):
        uart.kirim(packet.decode("latin-1"))
    uart.tutup()
    assert not ser.is_open
    assert all(not t.is_alive() for t in receiver.threads)
    assert receiver.csv_path.with_suffix(".bin").stat().st_size == 116 + 356


def test_receiver_corruption_resync_duplicate(tmp_path):
    ser = FakeSerial()
    receiver = DiagnosticReceiver(ser, tmp_path / "rx.csv").start()
    await_condition(lambda: receiver.ready)
    packet = encode_detection_packet(20, 1, 1, 2, 3, 4)
    assert receiver.submit(packet)
    await_condition(lambda: len(read_csv(receiver.csv_path)) == 4)
    good = encode_record(
        CommandResult(
            Header(
                record_seq=2,
                command_seq=2,
                timestamp_ms=2,
                obj_id=20,
                class_id=1,
                command_type=1,
                rx_x_01mm=10,
                rx_y_01mm=20,
                rx_angle_01deg=30,
                rx_corr_01deg=40,
                status_bits=0x801,
            )
        )
    )
    bad = bytearray(good)
    bad[-4] ^= 1
    with ser.lock:
        ser.rx.extend(b"noise\xd3\x7e\xff" + bytes(bad) + good)
    await_condition(lambda: receiver.health["duplicate"] == 1)
    receiver.close()
    assert receiver.health["decoder_errors"] >= 2
    assert read_csv(receiver.csv_path)[-1]["match_status"] == "duplicate"


def test_receiver_short_write_fault(tmp_path):
    ser = FakeSerial()
    ser.short = True
    receiver = DiagnosticReceiver(ser, tmp_path / "fault.csv").start()
    await_condition(lambda: receiver.fault is not None)
    assert not receiver.submit(encode_detection_packet(1, 1, 0, 0, 0, 0))
    receiver.close()
    assert receiver.sent_failures == 1


def test_pending_id_reuse_blocked(tmp_path):
    ser = FakeSerial()
    receiver = DiagnosticReceiver(ser, tmp_path / "ids.csv").start()
    await_condition(lambda: receiver.ready)
    ser.delay = 0.2
    packet = encode_detection_packet(65535, 1, 1, 2, 3, 4)
    assert receiver.submit(packet)
    assert not receiver.submit(packet)
    receiver.close()


def test_slow_disk_does_not_block_admission_and_overflow_fault(tmp_path):
    block = threading.Event()

    class SlowWriter:
        def __init__(self, path):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def write(self, rows):
            block.wait(1)

    ser = FakeSerial()
    receiver = DiagnosticReceiver(
        ser, tmp_path / "slow.csv", capacity=1, writer_factory=SlowWriter
    ).start()
    # READY is updated before disk write; the logger then deliberately stalls.
    await_condition(lambda: receiver.ready)
    start = time.monotonic()
    assert receiver.submit(encode_detection_packet(1, 1, 0, 0, 0, 0))
    assert time.monotonic() - start < 0.05
    await_condition(lambda: len(ser.frames) == 2)
    with ser.lock:
        ser.rx.extend(
            encode_record(CommandResult(Header(record_seq=3, timestamp_ms=3)))
        )
    await_condition(lambda: receiver.fault is not None)
    assert receiver.overflows > 0
    block.set()
    receiver.close()
    assert all(not t.is_alive() for t in receiver.threads)


@pytest.mark.parametrize("class_id", [1, 2])
def test_full_runtime_angle_sweep_signed_correction(class_id):
    for units in range(3600):
        angle, _, _, correction, *_ = evaluasi_qc_dan_servo(
            class_id, "elco_benar", sudut_manual=units / 10
        )
        frame = encode_runtime_detection_packet(
            units, class_id, 1, 2, angle, correction
        )
        decoded = decode_command(frame)
        assert decoded.angle_01deg == units
        expected = (-units) % 3600
        if expected > 1800:
            expected -= 3600
        assert decoded.corr_01deg == expected
        assert -1800 <= decoded.corr_01deg <= 1800
    assert (
        decode_command(
            encode_runtime_detection_packet(1, class_id, 0, 0, 359.95, 180)
        ).angle_01deg
        == 0
    )
    assert (
        decode_command(
            encode_runtime_detection_packet(1, class_id, 0, 0, 180, 180)
        ).corr_01deg
        == 1800
    )
    assert (
        decode_command(
            encode_runtime_detection_packet(1, class_id, 0, 0, 90, 270)
        ).corr_01deg
        == -900
    )


def test_invalid_frame_admissions_never_raise(tmp_path, caplog):
    receiver = DiagnosticReceiver(FakeSerial(), tmp_path / "invalid.csv")
    receiver.ready = True
    bad_frames = [
        encode_detection_packet(1, 3, 0, 0, 0, 0),
        encode_detection_packet(1, 255, 0, 0, 0, 0),
        encode_detection_packet(1, 1, 0, 0, 90, 270),
        encode_detection_packet(1, 1, 0, 0, 359.95, 0),
        b"truncated",
        "not binary",
    ]
    for frame in bad_frames:
        assert not receiver.submit(frame)
    assert receiver.admission_failures == len(bad_frames)
    assert receiver.tx.empty()
    assert "UART admission rejected" in caplog.text


def test_runtime_class_edge_is_counted_without_crashing(tmp_path, monkeypatch, caplog):
    monkeypatch.setattr(KoneksiUART, "_connect", lambda self: None)
    uart = KoneksiUART(csv_path=tmp_path / "classes.csv")
    uart.ser = FakeSerial()
    uart.receiver = DiagnosticReceiver(uart.ser, uart.csv_path)
    uart.receiver.ready = True
    for class_id in (3, 255, 256, -1):
        assert not uart.kirim_deteksi(1, class_id, 0, 0, 90, 270, force=True)
    assert uart.receiver.admission_failures == 4
    assert uart.kirim_deteksi(1, 2, 0, 0, 359.95, 270, force=True)
    frame, _ = uart.receiver.tx.get_nowait()
    assert decode_command(frame).angle_01deg == 0
    assert decode_command(frame).corr_01deg == -900
    assert "unsupported vision class ID" in caplog.text


@pytest.mark.parametrize("continuous_noise", [False, True])
def test_dropped_heartbeat_rotates_expires_and_recovers(tmp_path, continuous_noise):
    ser = FakeSerial()
    ser.drop_heartbeats = 1
    ser.continuous_noise = continuous_noise
    receiver = DiagnosticReceiver(ser, tmp_path / "heartbeat.csv", timeout=0.1).start()
    try:
        await_condition(lambda: receiver.ready, timeout=3)
        commands = [decode_command(frame) for frame in ser.frames]
        assert len(commands) == 2
        assert commands[0].obj_id != commands[1].obj_id
        assert receiver.correlator.sent[1].state == "expired"
        assert receiver.health["missing"] == 1
        assert not receiver.fault
    finally:
        receiver.close()


def test_unsolicited_event_status_keeps_ready_and_fault_disables(tmp_path):
    ser = FakeSerial()
    receiver = DiagnosticReceiver(ser, tmp_path / "status.csv").start()
    try:
        await_condition(lambda: receiver.ready)
        event = Status(
            Header(
                record_type=0x83,
                payload_len=72,
                record_seq=2,
                timestamp_ms=2,
                status_bits=0x1800,
            ),
            StatusPayload(state=2, homed_mask=3),
        )
        with ser.lock:
            ser.rx.extend(encode_record(event))
        await_condition(lambda: len(read_csv(receiver.csv_path)) == 2)
        assert receiver.ready
        fault = replace(
            event,
            header=replace(
                event.header, record_seq=3, timestamp_ms=3, status_bits=0x40
            ),
        )
        with ser.lock:
            ser.rx.extend(encode_record(fault))
        await_condition(lambda: not receiver.ready)
    finally:
        receiver.close()


def test_rejected_pick_does_not_pause_next_object_for_heartbeat(tmp_path):
    ser = FakeSerial()
    ser.reject_pick_ids.add(1)
    receiver = DiagnosticReceiver(ser, tmp_path / "rejected_pick.csv").start()
    try:
        await_condition(lambda: receiver.ready)
        assert receiver.submit(encode_detection_packet(1, 1, 999, 0, 0, 0))
        await_condition(lambda: len(read_csv(receiver.csv_path)) == 4)
        rejected_rows = read_csv(receiver.csv_path)[1:]
        assert all(row["match_status"] == "rejected" for row in rejected_rows)
        assert receiver.health["rejected"] == 1
        assert receiver.health["failed_command_results"] == 1
        assert receiver.ready
        assert receiver.submit(encode_detection_packet(2, 1, 1, 0, 0, 0))
        await_condition(lambda: len(read_csv(receiver.csv_path)) == 7)
        assert [decode_command(frame).command_type for frame in ser.frames] == [2, 1, 1]
        assert receiver.ready
        assert read_csv(receiver.csv_path)[-1]["match_status"] == "matched"
    finally:
        receiver.close()


def test_home_result_updates_readiness(tmp_path):
    ser = FakeSerial()
    receiver = DiagnosticReceiver(ser, tmp_path / "home_readiness.csv").start()
    try:
        await_condition(lambda: receiver.ready)
        failed_home = HomeResult(
            Header(
                record_type=0x82,
                payload_len=80,
                record_seq=2,
                timestamp_ms=2,
                status_bits=0x24000,
            ),
            (HomeAxis(result=2), HomeAxis(axis=1, result=2)),
        )
        with ser.lock:
            ser.rx.extend(encode_record(failed_home))
        await_condition(lambda: len(read_csv(receiver.csv_path)) == 3)
        assert not receiver.ready
        successful_home = HomeResult(
            replace(
                failed_home.header,
                record_seq=3,
                timestamp_ms=3,
                status_bits=1,
                home_epoch=1,
            ),
            (
                HomeAxis(result=1, axis_flags=0x38, as_status=0x20),
                HomeAxis(axis=1, result=1, axis_flags=0x38, as_status=0x20),
            ),
        )
        with ser.lock:
            ser.rx.extend(encode_record(successful_home))
        await_condition(lambda: receiver.ready)
    finally:
        receiver.close()
