"""Version-one LE records; CRC excludes only the diagnostic preamble."""

import struct
from dataclasses import astuple, dataclass, field

from .crc import crc16_ccitt

PREAMBLE = b"\xd3\x7e"
LENGTHS = {0x81: 312, 0x82: 80, 0x83: 72}
HEADER = struct.Struct("<BBHIIIIIHBBhhHhBB")
PHASE = struct.Struct("<BBHiiiiiiIIiHHHHBBHHBBBBHiiiiiiIIIIHH")
HOME = struct.Struct("<BBBBiIiHHHHiiIHBB")
STATUS = struct.Struct("<BBBBIIIIIIIiiiiIIBBBBiiHH")


@dataclass(frozen=True)
class Header:
    version: int = 1
    record_type: int = 0x81
    payload_len: int = 312
    record_seq: int = 0
    timestamp_ms: int = 0
    command_seq: int = 0
    home_epoch: int = 0
    status_bits: int = 0
    obj_id: int = 0
    class_id: int = 0
    command_type: int = 0
    rx_x_01mm: int = 0
    rx_y_01mm: int = 0
    rx_angle_01deg: int = 0
    rx_corr_01deg: int = 0
    phase_count: int = 0
    config_id: int = 1


@dataclass(frozen=True)
class Phase:
    axis: int = 0
    phase: int = 1
    axis_flags: int = 0
    requested_target_01mm: int = 0
    applied_target_01mm: int = 0
    start_position_steps: int = 0
    target_position_steps: int = 0
    commanded_delta_steps: int = 0
    emitted_delta_steps: int = 0
    emitted_edge_count: int = 0
    step_angle_udeg: int = 112500
    quant_residual_nm: int = 0
    mscnt_start: int = 0
    mscnt_end: int = 0
    mscnt_expected: int = 0
    mscnt_observed: int = 0
    mscnt_check: int = 0
    driver_mode: int = 4
    raw_start: int = 0
    raw_end: int = 0
    as_status_start: int = 0
    as_status_end: int = 0
    agc_start: int = 0
    agc_end: int = 0
    as_conf: int = 0
    unwrap_start: int = 0
    unwrap_end: int = 0
    unwrap_delta: int = 0
    shaft_delta_001deg: int = 0
    displacement_um: int = 0
    home_offset_counts: int = 0
    move_duration_us: int = 0
    max_sample_gap_us: int = 0
    raw_start_timestamp_us: int = 0
    raw_end_timestamp_us: int = 0
    health_start_age_ms: int = 0
    health_end_age_ms: int = 0


@dataclass(frozen=True)
class HomeAxis:
    axis: int = 0
    result: int = 0
    reserved: int = 0
    sg_threshold: int = 0
    seek_emitted_steps: int = 0
    backoff_steps: int = 0
    latch_emitted_steps: int = 0
    sg_baseline: int = 0
    sg_trigger: int = 0
    mscnt_zero: int = 0
    raw_zero: int = 0
    home_offset_counts: int = 0
    home_emitted_origin_steps: int = 0
    home_duration_us: int = 0
    axis_flags: int = 0
    as_status: int = 0
    agc: int = 0


@dataclass(frozen=True)
class StatusPayload:
    state: int = 0
    command_queue_depth: int = 0
    tx_queue_depth: int = 0
    homed_mask: int = 0
    rx_crc_errors: int = 0
    rx_format_errors: int = 0
    rx_overflows: int = 0
    rejected_commands: int = 0
    tx_record_drops: int = 0
    x_drv_status: int = 0
    z_drv_status: int = 0
    x_position_steps: int = 0
    z_position_steps: int = 0
    x_unwrap_counts: int = 0
    z_unwrap_counts: int = 0
    x_max_sample_gap_us: int = 0
    z_max_sample_gap_us: int = 0
    x_as_status: int = 0
    z_as_status: int = 0
    x_agc: int = 0
    z_agc: int = 0
    x_home_offset_counts: int = 0
    z_home_offset_counts: int = 0
    x_health_age_ms: int = 65535
    z_health_age_ms: int = 65535


@dataclass(frozen=True)
class CommandResult:
    header: Header = field(default_factory=Header)
    phases: tuple[Phase, ...] = field(
        default_factory=lambda: (
            Phase(),
            Phase(axis=1, phase=2),
            Phase(axis=1, phase=3),
        )
    )


@dataclass(frozen=True)
class HomeResult:
    header: Header = field(
        default_factory=lambda: Header(record_type=0x82, payload_len=80)
    )
    axes: tuple[HomeAxis, ...] = field(
        default_factory=lambda: (HomeAxis(), HomeAxis(axis=1))
    )


@dataclass(frozen=True)
class Status:
    header: Header = field(
        default_factory=lambda: Header(record_type=0x83, payload_len=72)
    )
    status: StatusPayload = field(default_factory=StatusPayload)


Record = CommandResult | HomeResult | Status


def encode_record(record: Record) -> bytes:
    h = record.header
    if isinstance(record, CommandResult):
        if len(record.phases) != 3 or h.record_type != 0x81:
            raise ValueError("COMMAND_RESULT requires three phases")
        payload = b"".join(PHASE.pack(*astuple(p)) for p in record.phases)
    elif isinstance(record, HomeResult):
        if len(record.axes) != 2 or h.record_type != 0x82:
            raise ValueError("HOME_RESULT requires two axes")
        payload = b"".join(HOME.pack(*astuple(p)) for p in record.axes)
    else:
        if h.record_type != 0x83:
            raise ValueError("STATUS type mismatch")
        payload = STATUS.pack(*astuple(record.status))
    if (
        h.version != 1
        or LENGTHS[h.record_type] != len(payload)
        or h.payload_len != len(payload)
    ):
        raise ValueError("invalid v1 header")
    body = HEADER.pack(*astuple(h)) + payload
    return PREAMBLE + body + struct.pack("<H", crc16_ccitt(body)) + b"\r\n"


def decode_record(frame: bytes) -> Record:
    if len(frame) < 44 or frame[:2] != PREAMBLE:
        raise ValueError("diagnostic framing")
    h = Header(*HEADER.unpack_from(frame, 2))
    if h.version != 1 or LENGTHS.get(h.record_type) != h.payload_len:
        raise ValueError("unsupported version/type/length")
    if len(frame) != 44 + h.payload_len or frame[-2:] != b"\r\n":
        raise ValueError("diagnostic length/tail")
    if crc16_ccitt(frame[2:-4]) != struct.unpack_from("<H", frame, len(frame) - 4)[0]:
        raise ValueError("diagnostic CRC")
    if h.record_type == 0x81:
        return CommandResult(
            h, tuple(Phase(*PHASE.unpack_from(frame, b)) for b in (40, 144, 248))
        )
    if h.record_type == 0x82:
        return HomeResult(
            h, tuple(HomeAxis(*HOME.unpack_from(frame, b)) for b in (40, 80))
        )
    return Status(h, StatusPayload(*STATUS.unpack_from(frame, 40)))


class StreamDecoder:
    """At most one partial 356-byte candidate retained between feeds."""

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0
        self.discarded_bytes = 0
        self.record_gaps = 0
        self.last_record_seq = None

    def feed(self, data: bytes) -> list[Record]:
        self.buffer.extend(data)
        records = []
        while True:
            i = self.buffer.find(PREAMBLE)
            if i < 0:
                keep = int(self.buffer.endswith(PREAMBLE[:1]))
                self.discarded_bytes += len(self.buffer) - keep
                del self.buffer[: len(self.buffer) - keep]
                break
            if i:
                self.discarded_bytes += i
                del self.buffer[:i]
            if len(self.buffer) < 6:
                break
            version, kind, length = struct.unpack_from("<BBH", self.buffer, 2)
            if version != 1 or LENGTHS.get(kind) != length:
                self.errors += 1
                del self.buffer[0]
                continue
            size = length + 44
            if len(self.buffer) < size:
                break
            try:
                record = decode_record(bytes(self.buffer[:size]))
            except ValueError:
                self.errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            if self.last_record_seq is not None:
                delta = (record.header.record_seq - self.last_record_seq) & 0xFFFFFFFF
                if 1 < delta < 0x80000000:
                    self.record_gaps += delta - 1
            self.last_record_seq = record.header.record_seq
            records.append(record)
        return records
