#!/usr/bin/env python3
"""Interactive serial console and live plot for the rail drive board."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import math
import queue
import re
import sys
import threading
import time
from typing import Any


BAUD_RATE = 230400
TRANSACTION_PERIOD_S = 0.002
REPLY_TIMEOUT_S = 0.050
CALIBRATION_TIMEOUT_S = 60.0
SHUTDOWN_TIMEOUT_S = 1.0
HISTORY_SECONDS = 30.0
PLOT_PERIOD_MS = 50
STATUS_PERIOD_S = 1.0
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


def parse_ack(reply: str) -> tuple[float, float] | None:
    """Return position and velocity from either supported ACK shape."""
    match = ACK_RE.fullmatch(reply)
    if match is None:
        return None
    try:
        values = tuple(float(field) for field in match.groups() if field is not None)
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values[0], values[1]


class SharedState:
    """Small synchronized state shared by serial, input, and GUI threads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.samples: deque[tuple[float, float, float]] = deque()
        self.mode = "starting"
        self.latest: tuple[float, float] | None = None
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

    def clear_history(self) -> None:
        with self.lock:
            self.samples.clear()
            self.latest = None

    def add_sample(self, position: float, velocity: float) -> None:
        elapsed = time.monotonic() - self.started
        with self.lock:
            self.samples.append((elapsed, position, velocity))
            cutoff = elapsed - HISTORY_SECONDS
            while self.samples and self.samples[0][0] < cutoff:
                self.samples.popleft()
            self.latest = (position, velocity)
            self.fault = None

    def mark_failed(self) -> None:
        with self.lock:
            self.failed = True

    def snapshot(
        self,
    ) -> tuple[
        float,
        list[float],
        list[float],
        list[float],
        str,
        tuple[float, float] | None,
        str | None,
    ]:
        elapsed = time.monotonic() - self.started
        cutoff = elapsed - HISTORY_SECONDS
        with self.lock:
            samples = [sample for sample in self.samples if sample[0] >= cutoff]
            mode = self.mode
            latest = self.latest
            fault = self.fault
        times = [sample[0] for sample in samples]
        positions = [sample[1] for sample in samples]
        velocities = [sample[2] for sample in samples]
        return elapsed, times, positions, velocities, mode, latest, fault


def print_help() -> None:
    emit(
        "Commands: dis | cal | sp <mm/s> | kd <gain> | help | quit (or q)\n"
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
        elif command.kind == "quit":
            shared.quit_requested.set()
            return
        else:
            commands.put(command)


def transact(port: Any, request: str) -> str:
    """Perform one bounded, newline-delimited request/reply transaction."""
    payload = request.encode("ascii")
    written = port.write(payload)
    if written != len(payload):
        raise OSError("incomplete serial write")

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


def update_terminal_status(shared: SharedState) -> None:
    _, _, _, _, mode, latest, fault = shared.snapshot()
    if mode == "active" and latest is not None:
        detail = f"pos={latest[0]:.1f} mm  vel={latest[1]:.1f} mm/s"
    else:
        detail = "pos=--  vel=-- (no telemetry)"
    if fault is not None:
        detail += f"  fault={fault}"
    emit(f"[{mode}] {detail}")


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
        emit("command: continuously streaming dis 0")
        return "dis", False, Decimal("0.0"), None, None

    if command.kind == "cal":
        if mode != "dis":
            emit("cal rejected: enter 'dis' first and allow the drive to stop")
            return mode, calibrated, setpoint, None, None
        shared.clear_history()
        shared.set_mode("calibrating")
        emit("command: calibrating (60 second timeout)")
        return "cal", False, setpoint, time.monotonic(), None

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


def request_for(mode: str, setpoint: Decimal) -> str:
    if mode == "cal":
        return "cal 0\n"
    if mode == "sp":
        return format_setpoint(setpoint)
    return "dis 0\n"


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
            emit(f"firmware returned {reply}; switching to dis 0")
        return "dis", calibrated, Decimal("0.0"), True

    if mode == "dis":
        if reply != "dis 0":
            return mode, calibrated, setpoint, False
        shared.set_mode("disabled")
        return mode, calibrated, setpoint, True

    if mode == "cal":
        if reply == "cal 0":
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
            if transact(port, "dis 0\n") == "dis 0":
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
    calibration_started: float | None = None
    next_request = time.monotonic()
    next_status = next_request + STATUS_PERIOD_S

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
        shared.set_mode("disabled")
        emit(f"opened {port_path} at {BAUD_RATE} baud; streaming dis 0")

        while not shared.quit_requested.is_set():
            old_mode = mode
            mode, calibrated, setpoint, new_calibration, one_shot = apply_command(
                commands, shared, mode, calibrated, setpoint
            )
            if new_calibration is not None:
                calibration_started = new_calibration
            elif mode != old_mode and mode != "cal":
                calibration_started = None

            now = time.monotonic()
            if (
                mode == "cal"
                and calibration_started is not None
                and now - calibration_started >= CALIBRATION_TIMEOUT_S
            ):
                emit("calibration timed out; switching to dis 0")
                mode = "dis"
                calibrated = False
                setpoint = Decimal("0.0")
                calibration_started = None
                shared.set_mode("disabled", fault="calibration timeout")

            request = one_shot if one_shot is not None else request_for(mode, setpoint)
            try:
                reply = transact(port, request)
            except (OSError, TimeoutError, ValueError) as error:
                emit(f"serial communication failed: {error}")
                communication_failed = True
                break

            if one_shot is not None:
                valid = reply == one_shot.strip()
                if valid:
                    emit(f"gain updated: {reply}")
            else:
                mode, calibrated, setpoint, valid = process_reply(
                    reply, shared, mode, calibrated, setpoint
                )
            if not valid:
                emit(f"unexpected reply to {request.strip()!r}: {reply!r}")
                communication_failed = True
                break
            if mode != "cal":
                calibration_started = None

            now = time.monotonic()
            if now >= next_status:
                update_terminal_status(shared)
                next_status = now + STATUS_PERIOD_S

            next_request += TRANSACTION_PERIOD_S
            now = time.monotonic()
            if next_request <= now:
                missed = math.floor((now - next_request) / TRANSACTION_PERIOD_S) + 1
                next_request += missed * TRANSACTION_PERIOD_S
            shared.quit_requested.wait(max(0.0, next_request - now))

        if communication_failed:
            try:
                transact(port, "dis 0\n")
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


def load_dependencies() -> tuple[Any, Any]:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("PySerial is required (install package 'pyserial')") from error
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("Matplotlib is required (install package 'matplotlib')") from error

    backend = matplotlib.get_backend().lower()
    noninteractive = {name.lower() for name in matplotlib.rcsetup.non_interactive_bk}
    if backend in noninteractive:
        raise RuntimeError(
            f"Matplotlib backend {matplotlib.get_backend()!r} is not interactive; "
            "a graphical display is required"
        )
    return serial, plt


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Continuously command and visualize the rail drive board."
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
        serial_module, plt = load_dependencies()
        figure, (position_axis, velocity_axis) = plt.subplots(
            2, 1, sharex=True, figsize=(10, 7)
        )
    except Exception as error:
        print(f"serial-drive: {error}", file=sys.stderr)
        return 1

    figure.canvas.manager.set_window_title("Rail drive position and velocity")
    figure.suptitle("Rail drive: starting")
    position_line, = position_axis.plot([], [], color="tab:blue")
    velocity_line, = velocity_axis.plot([], [], color="tab:orange")
    position_axis.set_ylabel("Position (mm)")
    position_axis.set_ylim(-10.0, 510.0)
    position_axis.grid(True)
    velocity_axis.set_ylabel("Velocity (mm/s)")
    velocity_axis.set_xlabel("Elapsed time (s)")
    velocity_axis.set_ylim(-35.0, 35.0)
    velocity_axis.grid(True)

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

    def update_plot() -> bool:
        elapsed, times, positions, velocities, mode, _, fault = shared.snapshot()
        position_line.set_data(times, positions)
        velocity_line.set_data(times, velocities)
        right = max(HISTORY_SECONDS, elapsed)
        left = max(0.0, right - HISTORY_SECONDS)
        velocity_axis.set_xlim(left, right)
        title = f"Rail drive: {mode}"
        if mode != "active":
            title += " (no telemetry)"
        if fault is not None:
            title += f" — {fault}"
        figure.suptitle(title)
        figure.canvas.draw_idle()
        if shared.finished.is_set():
            plt.close(figure)
            return False
        return True

    def close_window(_event: Any) -> None:
        shared.quit_requested.set()

    figure.canvas.mpl_connect("close_event", close_window)
    timer = figure.canvas.new_timer(interval=PLOT_PERIOD_MS)
    timer.add_callback(update_plot)
    timer.start()

    print_help()
    serial_thread.start()
    terminal_thread.start()
    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        emit("interrupt received; shutting down")
    finally:
        shared.quit_requested.set()
        plt.close(figure)
        serial_thread.join(timeout=SHUTDOWN_TIMEOUT_S + REPLY_TIMEOUT_S + 1.0)

    if serial_thread.is_alive():
        emit("serial worker did not stop cleanly")
        shared.mark_failed()
    return 1 if shared.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
