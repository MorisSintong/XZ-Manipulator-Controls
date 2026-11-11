"""Shared outcome/fault classification for diagnostic v1 status bits."""

from enum import IntFlag


class StatusBits(IntFlag):
    SUCCESS = 1 << 0
    ABORTED = 1 << 1
    CANCELLED = 1 << 2
    REJECTED = 1 << 3
    CLAMPED = 1 << 4
    NOT_HOMED = 1 << 5
    DRIVER_FAULT = 1 << 6
    ENCODER_INVALID = 1 << 7
    SAMPLE_OVERRUN = 1 << 8
    STEP_TIMING_FAULT = 1 << 9
    MSCNT_MISMATCH = 1 << 10
    MSCNT_UNQUALIFIED = 1 << 11
    RX_OVERFLOW = 1 << 12
    TX_BACKPRESSURE = 1 << 13
    HOME_FAILED = 1 << 14
    LIMIT_FAULT = 1 << 15
    CONFIG_INVALID = 1 << 16
    POSITION_UNCERTAIN = 1 << 17


FAULT_BITS = (
    StatusBits.ABORTED
    | StatusBits.NOT_HOMED
    | StatusBits.DRIVER_FAULT
    | StatusBits.ENCODER_INVALID
    | StatusBits.SAMPLE_OVERRUN
    | StatusBits.STEP_TIMING_FAULT
    | StatusBits.MSCNT_MISMATCH
    | StatusBits.TX_BACKPRESSURE
    | StatusBits.HOME_FAILED
    | StatusBits.LIMIT_FAULT
    | StatusBits.CONFIG_INVALID
    | StatusBits.POSITION_UNCERTAIN
)


def has_fault(status_bits: int) -> bool:
    return bool(status_bits & FAULT_BITS)


def command_succeeded(status_bits: int) -> bool:
    return bool(status_bits & StatusBits.SUCCESS) and not has_fault(status_bits)


def host_ready(homed_mask: int, status_bits: int) -> bool:
    return homed_mask & 3 == 3 and not has_fault(status_bits)
