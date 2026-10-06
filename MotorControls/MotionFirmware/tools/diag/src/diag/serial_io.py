import time
from pathlib import Path

import serial
from motion_diagnostics.commands import control_packet, decode_command
from motion_diagnostics.correlation import Correlator
from motion_diagnostics.csv_io import CsvWriter, record_rows
from motion_diagnostics.frames import StreamDecoder


def open_serial(port):
    ser = serial.serial_for_url(
        port, baudrate=115200, timeout=0.05, write_timeout=0.2, do_not_open=True
    )
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class SerialSession:
    def __init__(self, port, csv_path, config, timeout=10, serial_factory=open_serial):
        self.ser = serial_factory(port)
        try:
            self.writer = CsvWriter(csv_path)
            try:
                self.raw = Path(csv_path).with_suffix(".bin").open("wb")
            except OSError:
                self.writer.close()
                raise
        except OSError:
            self.ser.close()
            raise
        self.correlator = Correlator(timeout=timeout)
        self.decoder = StreamDecoder()
        self.config = config
        self.timeout = timeout

    def send(self, packet, **metadata):
        sent = self.correlator.register(decode_command(packet), **metadata)
        try:
            if self.ser.write(packet) != len(packet):
                raise OSError("short serial write: uncertain execution")
            self.correlator.mark_sent(sent.host_send_seq)
        except Exception:
            self.correlator.mark_sent(sent.host_send_seq, False)
            raise
        return sent

    def poll(self):
        data = self.ser.read(512)
        self.raw.write(data)
        self.raw.flush()
        self.correlator.expire()
        results = []
        for record in self.decoder.feed(data):
            context = self.correlator.match(record)
            self.writer.write(record_rows(record, context, self.config))
            results.append((record, context))
        return results

    def wait(self, sent):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            for record, context in self.poll():
                if context.get("host_send_seq") == sent.host_send_seq and context[
                    "match_status"
                ] in ("matched", "rejected"):
                    return record
            time.sleep(0.001)
        self.correlator.expire(float("inf"))
        # Never retry the uncertain command. Best-effort software abort only.
        try:
            self.send(control_packet(3, 65535))
        except (OSError, ValueError, RuntimeError) as exc:
            print(f"software abort transmission failed: {exc}")
        raise TimeoutError(
            "terminal result missing; abort attempted; execution uncertain"
        )

    def close(self):
        self.correlator.expire(float("inf"))
        try:
            self.writer.close()
        finally:
            try:
                self.raw.close()
            finally:
                self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
