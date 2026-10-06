"""Host-only joins; host sequence/session are never claimed to be wire echoes."""

import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone

from .commands import Command
from .status_bits import StatusBits, command_succeeded


def utc_now():
    return datetime.now(timezone.utc).isoformat()


@dataclass
class Sent:
    host_send_seq: int
    command: Command
    host_send_utc: str = field(default_factory=utc_now)
    registered_at: float = field(default_factory=time.monotonic)
    state: str = "pending"
    metadata: dict = field(default_factory=dict)
    completed: bool = False
    missing: bool = False
    firmware_floor: int | None = None


class Correlator:
    def __init__(self, session_id=None, timeout=10.0):
        self.session_id = session_id or str(uuid.uuid4())
        self.timeout = timeout
        self.lock = threading.RLock()
        self.next_seq = 0
        self.sent = {}
        self.seen = {}
        self.last_timestamp = None
        self.last_ordinal = None
        self.last_record_seq = None
        self.last_home_epoch = None
        self.timestamp_wraps = 0
        self.counts = {
            "missing": 0,
            "late": 0,
            "duplicate": 0,
            "unmatched": 0,
            "reset": 0,
            "commands_registered": 0,
            "commands_sent": 0,
            "scored_trials_registered": 0,
            "rejected": 0,
            "failed_command_results": 0,
        }

    def register(self, command: Command, **metadata) -> Sent:
        with self.lock:
            if len(self.sent) >= 4096:
                removable = [
                    k
                    for k, s in self.sent.items()
                    if s.completed or s.state in ("failed", "expired")
                ]
                if not removable:
                    raise ValueError("bounded command registry exhausted")
                del self.sent[removable[0]]
            if any(
                s.command.obj_id == command.obj_id
                and not s.completed
                and s.state not in ("failed", "expired")
                for s in self.sent.values()
            ):
                raise ValueError(
                    "object ID already outstanding (including timed-out commands)"
                )
            self.next_seq += 1
            sent = Sent(self.next_seq, command, metadata=metadata)
            sent.firmware_floor = self.last_ordinal
            self.sent[sent.host_send_seq] = sent
            self.counts["commands_registered"] += 1
            self.counts["scored_trials_registered"] += bool(metadata.get("scored"))
            return sent

    def mark_sent(self, seq, success=True):
        with self.lock:
            if seq not in self.sent or self.sent[seq].state == "expired":
                return False
            if success and self.sent[seq].state == "pending":
                self.counts["commands_sent"] += 1
            self.sent[seq].state = "sent" if success else "failed"
            return True

    def expire(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            expired = []
            for sent in self.sent.values():
                if (
                    not sent.completed
                    and not sent.missing
                    and sent.state != "failed"
                    and now - sent.registered_at > self.timeout
                ):
                    sent.missing = True
                    if sent.command.command_type in (2, 3, 4):
                        sent.state = "expired"
                    self.counts["missing"] += 1
                    expired.append(sent)
            return expired

    def reset(self):
        with self.lock:
            self.expire(float("inf"))
            self.sent.clear()
            self.seen.clear()
            self.session_id = str(uuid.uuid4())
            self.last_timestamp = self.last_ordinal = None
            self.last_record_seq = self.last_home_epoch = None
            self.timestamp_wraps = 0
            self.counts["reset"] += 1

    def match(self, record):
        h = record.header
        with self.lock:
            key = (h.command_seq, h.record_type)
            if h.command_seq and key in self.seen and self.seen[key][1] == record:
                self.counts["duplicate"] += 1
                return {
                    "session_id": self.session_id,
                    "run_id": self.session_id,
                    "host_receive_utc": utc_now(),
                    "match_status": "duplicate",
                    "timestamp_ms_extended": self.seen[key][2],
                }
            timestamp_reset = (
                self.last_timestamp is not None
                and h.timestamp_ms < self.last_timestamp
                and self.last_timestamp - h.timestamp_ms < 0x80000000
            )
            sequence_reset = (
                self.last_record_seq is not None
                and h.record_seq < self.last_record_seq
                and self.last_record_seq - h.record_seq < 0x80000000
            )
            epoch_reset = (
                self.last_home_epoch is not None and h.home_epoch < self.last_home_epoch
            )
            if timestamp_reset or sequence_reset or epoch_reset:
                self.reset()
            if self.last_timestamp is not None and h.timestamp_ms < self.last_timestamp:
                self.timestamp_wraps += 1
            self.last_timestamp = h.timestamp_ms
            self.last_record_seq = h.record_seq
            self.last_home_epoch = h.home_epoch
            base = {
                "session_id": self.session_id,
                "run_id": self.session_id,
                "host_receive_utc": utc_now(),
                "timestamp_ms_extended": h.timestamp_ms
                + self.timestamp_wraps * (1 << 32),
            }
            key = (h.command_seq, h.record_type)
            if h.command_seq and key in self.seen:
                self.counts["duplicate"] += 1
                return {**base, "match_status": "duplicate"}
            if not h.command_seq:
                return {**base, "match_status": "unsolicited"}
            matches = [
                s
                for s in self.sent.values()
                if not s.completed
                and s.state == "sent"
                and (
                    s.command.command_type,
                    s.command.obj_id,
                    s.command.class_id,
                    s.command.x_01mm,
                    s.command.y_01mm,
                    s.command.angle_01deg,
                    s.command.corr_01deg,
                )
                == (
                    h.command_type,
                    h.obj_id,
                    h.class_id,
                    h.rx_x_01mm,
                    h.rx_y_01mm,
                    h.rx_angle_01deg,
                    h.rx_corr_01deg,
                )
                and (
                    h.record_type
                    == {1: 0x81, 2: 0x83, 3: 0x83, 4: 0x82}[s.command.command_type]
                    or (
                        s.command.command_type == 4
                        and h.record_type == 0x83
                        and h.status_bits & StatusBits.REJECTED
                    )
                )
                and (
                    s.firmware_floor is None
                    or 0
                    < ((h.command_seq - s.firmware_floor) & 0xFFFFFFFF)
                    < 0x80000000
                )
            ]
            if len(matches) != 1:
                self.counts["unmatched"] += 1
                return {**base, "match_status": "ambiguous" if matches else "unmatched"}
            s = matches[0]
            s.completed = True
            if h.status_bits & StatusBits.REJECTED:
                self.counts["rejected"] += 1
            if h.record_type == 0x81 and not command_succeeded(h.status_bits):
                self.counts["failed_command_results"] += 1
            self.seen[key] = (s.host_send_seq, record, base["timestamp_ms_extended"])
            if len(self.seen) > 4096:
                del self.seen[next(iter(self.seen))]
            if (
                self.last_ordinal is None
                or 0 < ((h.command_seq - self.last_ordinal) & 0xFFFFFFFF) < 0x80000000
            ):
                self.last_ordinal = h.command_seq
            status = (
                "late"
                if s.missing
                else "rejected"
                if h.status_bits & StatusBits.REJECTED
                else "matched"
            )
            if s.missing:
                self.counts["late"] += 1
            return {
                **base,
                **s.metadata,
                "host_send_seq": s.host_send_seq,
                "host_send_utc": s.host_send_utc,
                "match_status": status,
            }
