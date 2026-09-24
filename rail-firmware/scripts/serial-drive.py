#!/usr/bin/env python3
"""Interactive serial debugger for the rail drive board."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import math
import queue
import re
import sys
import threading
import time
from typing import Any


BAUD_RATE = 115200
TRANSACTION_PERIOD_S = 0.004
REPLY_TIMEOUT_S = 0.050
SHUTDOWN_TIMEOUT_S = 1.0
MAX_REPLY_BYTES = 96
MAX_VELOCITY_MM_S = Decimal("32.0")
MAX_KD = Decimal("1.000")

ACK_NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
ACK_RE = re.compile(
    rf"^ack ({ACK_NUMBER}) ({ACK_NUMBER})(?: ({ACK_NUMBER}))?$"
)
SETPOINT_RE = re.compile(r"[+-]?\d+(?:\.\d+)?")
KD_RE = re.compile(r"[0-9]+(?:\.[0-9]{1,3})?")

KNOWN_ERRORS = {
    "err cal",
    "err dis",
    "err lim",
    "err hrd",
    "err est",
    "err pos",
    "err com",
    "err mot",
    "err can",
    "err cmd",
    "err kd",
    "err sys",
}

PRINT_LOCK = threading.Lock()


def emit(message: str) -> None:
    """Print a terminal event without interleaving worker output."""
    with PRINT_LOCK:
        print(message, flush=True)


@dataclass(frozen=True)
class UserCommand:
    kind: str
    value: Decimal | None = None


def parse_user_command(text: str) -> tuple[UserCommand | None, str | None]:
    """Parse one friendly terminal command."""
    command = text.strip()
    if not command:
        return None, None
    if command in {"quit", "q"}:
        return UserCommand("quit"), None
    if command == "help":
        return UserCommand("help"), None
    if command == "s":
        return UserCommand("status"), None
    if command == "dis":
        return UserCommand("dis"), None
    if command == "cal":
        return UserCommand("cal"), None

    fields = command.split()
    if len(fields) == 2 and fields[0] == "kd":
        if KD_RE.fullmatch(fields[1]) is None:
            return None, "Kd must be an unsigned decimal with up to 3 decimal places"
        try:
            value = Decimal(fields[1])
        except InvalidOperation:
            return None, "Kd must be an unsigned decimal with up to 3 decimal places"
        if value < 0 or value > MAX_KD:
            return None, "Kd must be between 0.000 and 1.000"
        return UserCommand("kd", value), None

    if len(fields) != 2 or fields[0] != "sp":
        return None, "invalid command; enter 'help' for usage"
    if SETPOINT_RE.fullmatch(fields[1]) is None:
        return None, "setpoint must be a plain decimal number in mm/s"

    try:
        value = Decimal(fields[1])
    except InvalidOperation:
        return None, "setpoint must be a plain decimal number in mm/s"
    if not value.is_finite():
        return None, "setpoint must be finite"
    if abs(value) > MAX_VELOCITY_MM_S:
        return None, "setpoint must be between -32.0 and 32.0 mm/s"
    try:
        quantized = value.quantize(Decimal("0.1"))
    except InvalidOperation:
        return None, "setpoint must be exactly representable to 0.1 mm/s"
    if value != quantized:
        return None, "setpoint must be exactly representable to 0.1 mm/s"
    if value == 0:
        value = Decimal("0.0")
    return UserCommand("sp", value), None


def format_setpoint(value: Decimal) -> str:
    return f"sp {value.quantize(Decimal('0.1')):.1f}\n"


def format_kd(value: Decimal) -> str:
    return f"kd {value.quantize(Decimal('0.001')):.3f}\n"


def parse_ack(reply: str) -> tuple[float, float, float | None] | None:
    """Return position, velocity, and optional acceleration from an ACK."""
    match = ACK_RE.fullmatch(reply)
    if match is None:
        return None
    try:
        values = tuple(float(field) for field in match.groups() if field is not None)
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    acceleration = values[2] if len(values) == 3 else None
    return values[0], values[1], acceleration


class SharedState:
    """Small synchronized state shared by serial and input threads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.mode = "starting"
        self.latest: tuple[float, float, float | None] | None = None
        self.fault: str | None = None
        self.failed = False
        self.quit_requested = threading.Event()
        self.finished = threading.Event()

    def set_mode(self, mode: str, *, fault: str | None = None) -> bool:
        with self.lock:
            changed = self.mode != mode or self.fault != fault
            self.mode = mode
            self.fault = fault
        return changed

    def clear_sample(self) -> None:
        with self.lock:
            self.latest = None

    def add_sample(
        self, position: float, velocity: float, acceleration: float | None
    ) -> None:
        with self.lock:
            self.latest = (position, velocity, acceleration)
            self.fault = None

    def mark_failed(self) -> None:
        with self.lock:
            self.failed = True

    def snapshot(
        self,
    ) -> tuple[str, tuple[float, float, float | None] | None, str | None]:
        with self.lock:
            mode = self.mode
            latest = self.latest
            fault = self.fault
        return mode, latest, fault


def print_help() -> None:
    emit(
        "Commands: dis | cal | sp <mm/s> | kd <gain> | s | help | quit (or q)\n"
        "  s prints the latest position, velocity, acceleration, and fault state.\n"
        "  Setpoints must be within -32.0..32.0 and exactly representable to 0.1.\n"
        "  Kd must be within 0.000..1.000 with up to 3 decimal places."
    )


def input_loop(commands: queue.Queue[UserCommand], shared: SharedState) -> None:
    while not shared.quit_requested.is_set():
        try:
            text = input("drive> ")
        except EOFError:
            emit("stdin closed; shutting down")
            shared.quit_requested.set()
            return
        except KeyboardInterrupt:
            shared.quit_requested.set()
            return

        command, error = parse_user_command(text)
        if error is not None:
            emit(error)
        elif command is None:
            continue
        elif command.kind == "help":
            print_help()
        elif command.kind == "status":
            update_terminal_status(shared)
        elif command.kind == "quit":
            shared.quit_requested.set()
            return
        else:
            commands.put(command)


def read_reply(port: Any) -> str:
    """Read one bounded, newline-delimited reply."""
    deadline = time.monotonic() + REPLY_TIMEOUT_S
    reply = bytearray()
    while len(reply) <= MAX_REPLY_BYTES:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("reply timeout")
        port.timeout = remaining
        byte = port.read(1)
        if not byte:
            raise TimeoutError("reply timeout")
        if byte == b"\n":
            if not reply:
                raise ValueError("empty reply")
            try:
                return reply.decode("ascii")
            except UnicodeDecodeError as error:
                raise ValueError("non-ASCII reply") from error
        reply.extend(byte)
    raise ValueError("reply exceeds maximum length")


def transact(port: Any, request: str) -> str:
    """Perform one bounded, newline-delimited request/reply transaction."""
    payload = request.encode("ascii")
    written = port.write(payload)
    if written != len(payload):
        raise OSError("incomplete serial write")
    return read_reply(port)


def update_terminal_status(shared: SharedState) -> None:
    mode, latest, fault = shared.snapshot()
    if latest is not None:
        acceleration = (
            f"{latest[2]:.1f} mm/s^2" if latest[2] is not None else "--"
        )
        detail = (
            f"pos={latest[0]:.1f} mm  vel={latest[1]:.1f} mm/s  "
            f"acc={acceleration}"
        )
    else:
        detail = "pos=--  vel=--  acc=--"
    emit(f"[{mode}] {detail}  fault={fault or 'none'}")


def apply_command(
    commands: queue.Queue[UserCommand],
    shared: SharedState,
    mode: str,
    calibrated: bool,
    setpoint: Decimal,
) -> tuple[str, bool, Decimal, float | None, str | None]:
    try:
        command = commands.get_nowait()
    except queue.Empty:
        return mode, calibrated, setpoint, None, None

    if command.kind == "dis":
        shared.set_mode("disabled")
        emit("command: dis")
        return "dis", False, Decimal("0.0"), None, "dis\n"

    if command.kind == "cal":
        if mode != "dis":
            emit("cal rejected: enter 'dis' first and allow the drive to stop")
            return mode, calibrated, setpoint, None, None
        shared.clear_sample()
        shared.set_mode("calibrating")
        emit("command: calibrating (30 second firmware timeout)")
        return "cal", False, setpoint, None, "cal\n"

    if command.kind == "sp":
        if not calibrated:
            emit("setpoint rejected: run 'cal' first")
            return mode, calibrated, setpoint, None, None
        assert command.value is not None
        shared.set_mode("active")
        emit(f"command: continuously streaming {format_setpoint(command.value).strip()}")
        return "sp", calibrated, command.value, None, None

    if command.kind == "kd":
        assert command.value is not None
        request = format_kd(command.value)
        emit(f"command: setting {request.strip()}")
        return mode, calibrated, setpoint, None, request

    return mode, calibrated, setpoint, None, None


def request_for(mode: str, setpoint: Decimal) -> str | None:
    if mode == "sp":
        return format_setpoint(setpoint)
    return None


def process_reply(
    reply: str,
    shared: SharedState,
    mode: str,
    calibrated: bool,
    setpoint: Decimal,
) -> tuple[str, bool, Decimal, bool]:
    """Process a valid transaction reply; false means protocol failure."""
    if reply in KNOWN_ERRORS:
        if reply == "err pos" and mode == "sp":
            setpoint = Decimal("0.0")
            if shared.set_mode("active", fault=reply):
                emit("firmware returned err pos; holding sp 0.0")
            return "sp", calibrated, setpoint, True

        calibrated = False
        if shared.set_mode("disabled", fault=reply):
            emit(f"firmware returned {reply}; switching to dis")
        return "dis", calibrated, Decimal("0.0"), True

    if mode == "dis":
        if reply != "dis":
            return mode, calibrated, setpoint, False
        shared.set_mode("disabled")
        return mode, calibrated, setpoint, True

    if mode == "cal":
        if reply == "cal":
            shared.set_mode("calibrating")
            return mode, calibrated, setpoint, True
        sample = parse_ack(reply)
        if sample is None:
            return mode, calibrated, setpoint, False
        shared.add_sample(*sample)
        shared.set_mode("active")
        emit("calibration complete; holding sp 0.0")
        return "sp", True, Decimal("0.0"), True

    sample = parse_ack(reply)
    if sample is None:
        return mode, calibrated, setpoint, False
    shared.add_sample(*sample)
    shared.set_mode("active")
    return mode, calibrated, setpoint, True


def verify_disable(port: Any, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if transact(port, "dis\n") == "dis":
                return True
        except (OSError, TimeoutError, ValueError):
            pass
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(TRANSACTION_PERIOD_S, remaining))
    return False


def serial_loop(
    serial_module: Any,
    port_path: str,
    commands: queue.Queue[UserCommand],
    shared: SharedState,
) -> None:
    port = None
    communication_failed = False
    mode = "dis"
    calibrated = False
    setpoint = Decimal("0.0")
    next_request = time.monotonic()

    try:
        port = serial_module.Serial(
            port=port_path,
            baudrate=BAUD_RATE,
            bytesize=serial_module.EIGHTBITS,
            parity=serial_module.PARITY_NONE,
            stopbits=serial_module.STOPBITS_ONE,
            timeout=REPLY_TIMEOUT_S,
            write_timeout=REPLY_TIMEOUT_S,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        )
        port.reset_input_buffer()
        port.reset_output_buffer()
        if transact(port, "dis\n") != "dis":
            raise ValueError("drive did not acknowledge initial disable")
        shared.set_mode("disabled")
        emit(f"opened {port_path} at {BAUD_RATE} baud; drive disabled")

        while not shared.quit_requested.is_set():
            mode, calibrated, setpoint, _, one_shot = apply_command(
                commands, shared, mode, calibrated, setpoint
            )

            request = one_shot if one_shot is not None else request_for(mode, setpoint)
            try:
                if request is not None:
                    reply = transact(port, request)
                elif mode == "cal":
                    reply = read_reply(port)
                else:
                    shared.quit_requested.wait(TRANSACTION_PERIOD_S)
                    continue
            except TimeoutError as error:
                if mode == "cal" and request is None:
                    continue
                emit(f"serial communication failed: {error}")
                communication_failed = True
                break
            except (OSError, ValueError) as error:
                emit(f"serial communication failed: {error}")
                communication_failed = True
                break

            if one_shot is not None:
                if reply in KNOWN_ERRORS:
                    mode, calibrated, setpoint, valid = process_reply(
                        reply, shared, mode, calibrated, setpoint
                    )
                else:
                    valid = reply == one_shot.strip()
                if valid and one_shot.startswith("kd "):
                    emit(f"gain updated: {reply}")
            else:
                mode, calibrated, setpoint, valid = process_reply(
                    reply, shared, mode, calibrated, setpoint
                )
            if not valid:
                request_name = request.strip() if request is not None else "calibration"
                emit(f"unexpected reply to {request_name!r}: {reply!r}")
                communication_failed = True
                break

            next_request += TRANSACTION_PERIOD_S
            now = time.monotonic()
            if next_request <= now:
                missed = math.floor((now - next_request) / TRANSACTION_PERIOD_S) + 1
                next_request += missed * TRANSACTION_PERIOD_S
            shared.quit_requested.wait(max(0.0, next_request - now))

        if communication_failed:
            try:
                transact(port, "dis\n")
            except (OSError, TimeoutError, ValueError):
                pass
            shared.mark_failed()
        else:
            emit("verifying drive disable before closing")
            if verify_disable(port, SHUTDOWN_TIMEOUT_S):
                emit("drive disable acknowledged")
            else:
                emit("warning: drive disable could not be verified")
                shared.mark_failed()
    except (OSError, serial_module.SerialException, ValueError) as error:
        emit(f"could not open or configure {port_path}: {error}")
        shared.mark_failed()
    finally:
        if port is not None and port.is_open:
            port.close()
        shared.finished.set()


def load_dependencies() -> Any:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("PySerial is required (install package 'pyserial')") from error
    return serial


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Interactively debug the rail drive board."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="serial device (default: %(default)s)",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        serial_module = load_dependencies()
    except Exception as error:
        print(f"serial-drive: {error}", file=sys.stderr)
        return 1

    shared = SharedState()
    commands: queue.Queue[UserCommand] = queue.Queue()
    serial_thread = threading.Thread(
        target=serial_loop,
        args=(serial_module, args.port, commands, shared),
        name="serial-drive-io",
    )
    terminal_thread = threading.Thread(
        target=input_loop,
        args=(commands, shared),
        name="serial-drive-input",
        daemon=True,
    )

    print_help()
    serial_thread.start()
    terminal_thread.start()
    try:
        shared.finished.wait()
    except KeyboardInterrupt:
        emit("interrupt received; shutting down")
    finally:
        shared.quit_requested.set()
        serial_thread.join(timeout=SHUTDOWN_TIMEOUT_S + REPLY_TIMEOUT_S + 1.0)

    if serial_thread.is_alive():
        emit("serial worker did not stop cleanly")
        shared.mark_failed()
    return 1 if shared.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
