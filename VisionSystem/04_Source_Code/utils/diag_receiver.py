"""Bounded binary RX/TX/log workers; inference never touches serial or disk I/O."""

import json
import logging
import queue
import threading
import time
from pathlib import Path

from motion_diagnostics.commands import control_packet, decode_command
from motion_diagnostics.correlation import Correlator
from motion_diagnostics.csv_io import CsvWriter, record_rows
from motion_diagnostics.frames import HomeResult, Status, StreamDecoder
from motion_diagnostics.status_bits import command_succeeded, host_ready


class DiagnosticReceiver:
    def __init__(
        self,
        serial,
        csv_path,
        config=None,
        capacity=128,
        timeout=10.0,
        writer_factory=CsvWriter,
    ):
        if capacity < 1 or timeout <= 0:
            raise ValueError("positive bounded queue capacity and timeout required")
        self.serial = serial
        self.config = config or {}
        self.csv_path = Path(csv_path)
        self.correlator = Correlator(timeout=timeout)
        self.decoder = StreamDecoder()
        self.tx = queue.Queue(capacity)
        self.logs = queue.Queue(capacity)
        self.raw_chunks = queue.Queue(max(128, capacity))
        self.stop = threading.Event()
        self.fault = None
        self.ready = False
        self._heartbeat_at = 0.0
        self._heartbeat_id = 65534
        self.overflows = 0
        self.sent_failures = 0
        self.admission_failures = 0
        self.writer_factory = writer_factory
        self.threads = [
            threading.Thread(target=self._writer, name="diag-tx", daemon=True),
            threading.Thread(target=self._reader, name="diag-rx", daemon=True),
            threading.Thread(target=self._logger, name="diag-log", daemon=True),
        ]

    def start(self):
        self._heartbeat_at = time.monotonic()
        self._send_heartbeat()
        for thread in self.threads:
            thread.start()
        return self

    def _send_heartbeat(self):
        obj_id = self._heartbeat_id
        self._heartbeat_id = (self._heartbeat_id - 1) % 65536
        return self.submit(control_packet(2, obj_id), force_control=True)

    def reject_admission(self, reason):
        self.admission_failures += 1
        logging.getLogger(__name__).warning("UART admission rejected: %s", reason)
        return False

    def _fail(self, reason):
        self.fault = str(reason)
        self.ready = False

    def submit(self, frame: bytes, force_control=False, **metadata):
        try:
            if not isinstance(frame, bytes):
                raise TypeError("binary runtime accepts bytes only")
            command = decode_command(frame)
        except (ValueError, TypeError) as exc:
            return self.reject_admission(exc)
        if (
            self.stop.is_set()
            or self.fault
            or (command.command_type == 1 and not self.ready)
        ):
            return self.reject_admission(self.fault or "serial session is not ready")
        # Control traffic is allowed before READY; it does not clear a fault.
        if force_control and command.command_type == 1:
            return self.reject_admission("pick cannot bypass readiness")
        try:
            sent = self.correlator.register(command, **metadata)
            self.tx.put_nowait((frame, sent.host_send_seq))
            return True
        except queue.Full:
            self.correlator.mark_sent(sent.host_send_seq, False)
            self.overflows += 1
            self.admission_failures += 1
            self._fail("TX queue overflow")
        except ValueError as exc:
            self.reject_admission(exc)
            if "exhausted" in str(exc):
                self._fail(exc)
            return False
        return False

    def _writer(self):
        while not self.stop.is_set():
            try:
                frame, seq = self.tx.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                with self.correlator.lock:
                    belongs_to_session = seq in self.correlator.sent
                if not belongs_to_session or self.fault:
                    self.correlator.mark_sent(seq, False)
                    continue
                if self.serial.write(frame) != len(frame):
                    raise OSError("short serial write; execution uncertain")
                if not self.correlator.mark_sent(seq):
                    self._fail("session reset during write; execution uncertain")
            except Exception as exc:  # noqa: BLE001 - worker boundary must publish every failure
                self.correlator.mark_sent(seq, False)
                self.sent_failures += 1
                self._fail(exc)
            finally:
                self.tx.task_done()
        while True:
            try:
                _, seq = self.tx.get_nowait()
                self.correlator.mark_sent(seq, False)
                self.tx.task_done()
            except queue.Empty:
                break

    def _reader(self):
        try:
            while not self.stop.is_set():
                data = self.serial.read(512)
                self.correlator.expire()
                if (
                    not self.ready
                    and not self.fault
                    and time.monotonic() - self._heartbeat_at > 1
                ):
                    self._send_heartbeat()
                    self._heartbeat_at = time.monotonic()
                if not data:
                    time.sleep(0.001)
                    continue
                try:
                    self.raw_chunks.put_nowait(data)
                except queue.Full:
                    self.overflows += 1
                    self._fail("raw logging queue overflow")
                for record in self.decoder.feed(data):
                    try:
                        self.logs.put_nowait(record)
                    except queue.Full:
                        self.overflows += 1
                        self._fail(
                            "log queue overflow: accepted evidence may be missing"
                        )
        except Exception as exc:  # noqa: BLE001 - worker boundary must publish every failure
            if not self.stop.is_set():
                self._fail(exc)

    def _logger(self):
        try:
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            with (
                self.writer_factory(self.csv_path) as writer,
                self.csv_path.with_suffix(".bin").open("wb") as raw,
            ):
                while (
                    not self.stop.is_set()
                    or not self.logs.empty()
                    or not self.raw_chunks.empty()
                    or any(t.is_alive() for t in self.threads[:2])
                ):
                    while True:
                        try:
                            raw.write(self.raw_chunks.get_nowait())
                            self.raw_chunks.task_done()
                        except queue.Empty:
                            break
                    raw.flush()
                    try:
                        record = self.logs.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    # RX can precede write() returning. Resolve send state off inference.
                    deadline = time.monotonic() + 0.5
                    pending = False
                    while not self.stop.is_set():
                        with self.correlator.lock:
                            pending = any(
                                s.command.obj_id == record.header.obj_id
                                and s.state == "pending"
                                for s in self.correlator.sent.values()
                            )
                        if not pending or time.monotonic() >= deadline:
                            break
                        time.sleep(0.001)
                    context = self.correlator.match(record)
                    if pending:
                        self._fail("reply arrived but serial write state unresolved")
                    if (
                        context["match_status"] == "unmatched"
                        and self.correlator.counts["reset"]
                    ):
                        self.ready = False
                    if isinstance(record, Status):
                        healthy = host_ready(
                            record.status.homed_mask, record.header.status_bits
                        )
                        if not healthy:
                            self.ready = False
                        elif context["match_status"] == "matched":
                            self.ready = not self.fault
                    elif isinstance(record, HomeResult):
                        healthy = command_succeeded(record.header.status_bits) and all(
                            axis.result == 1 for axis in record.axes
                        )
                        if not healthy:
                            self.ready = False
                        elif context["match_status"] in ("matched", "unsolicited"):
                            self.ready = not self.fault
                    writer.write(record_rows(record, context, self.config))
                    self.logs.task_done()
        except Exception as exc:  # noqa: BLE001 - worker boundary must publish every failure
            self._fail(exc)

    def close(self, timeout=1.0):
        self.ready = False
        self.stop.set()
        for name in ("cancel_read", "cancel_write"):
            cancel = getattr(self.serial, name, None)
            if cancel:
                try:
                    cancel()
                except (OSError, RuntimeError) as exc:
                    self._fail(f"serial cancellation failed: {exc}")
        deadline = time.monotonic() + timeout
        for thread in self.threads:
            thread.join(max(0, deadline - time.monotonic()))
        if any(t.is_alive() for t in self.threads):
            self._fail("shutdown timeout: I/O worker did not terminate")
        self.correlator.expire(float("inf"))
        try:
            self.csv_path.with_suffix(".health.json").write_text(
                json.dumps(self.health, indent=2), encoding="utf-8"
            )
        except OSError as exc:
            self._fail(exc)

    @property
    def health(self):
        return {
            "ready": self.ready,
            "fault": self.fault,
            "queue_overflows": self.overflows,
            "send_failures": self.sent_failures,
            "admission_failures": self.admission_failures,
            "decoder_errors": self.decoder.errors,
            "record_gaps": self.decoder.record_gaps,
            **self.correlator.counts,
        }
