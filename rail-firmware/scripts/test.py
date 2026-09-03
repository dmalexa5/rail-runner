#!/usr/bin/env python3
"""Interact with the rail firmware's compact UART protocol."""

import argparse
import threading

import serial


FAULTS = {
    0: "none",
    1: "safety switch",
    2: "arm lease expired",
    3: "feedback timeout",
    4: "control-loop overrun",
    5: "CAN transmit failure",
}

STATUS_FIELDS = (
    "armed",
    "safe",
    "fault",
    "command torque",
    "control cycles",
    "control overruns",
    "maximum execution cycles",
    "arm lease age",
    "feedback age",
    "feedback sequence",
    "motor valid",
    "motor ID",
    "position",
    "velocity",
    "motor torque",
    "temperature",
    "motor error",
    "CAN error",
    "CAN TX OK",
    "CAN TX failed",
)


def translate(line):
    fields = line.strip().split(",")
    if fields == ["k", "a"]:
        return ["OK: armed; 100 ms lease refreshed"]
    if fields == ["k", "d"]:
        return ["OK: disarmed; torque reset to 0 Nm"]
    if fields == ["k", "t"]:
        return ["OK: torque command accepted"]
    if fields == ["e", "a"]:
        return ["ERROR: arm rejected"]
    if fields == ["e", "i"]:
        return ["ERROR: invalid torque value"]
    if fields == ["e", "r"]:
        return ["ERROR: torque command rejected"]
    if fields == ["e", "c"]:
        return ["ERROR: unknown command"]
    if len(fields) == 4 and fields[0] == "f":
        try:
            position, velocity, torque = (int(value) / 1000.0 for value in fields[1:])
        except ValueError:
            return ["Unrecognized response"]
        return [
            f"Feedback: position={position:g} rad, velocity={velocity:g} rad/s, "
            f"torque={torque:g} Nm"
        ]
    if len(fields) == 21 and fields[0] == "s":
        try:
            values = [int(value) for value in fields[1:18]]
            can_error = int(fields[18], 16)
            values.extend((can_error, int(fields[19]), int(fields[20])))
        except ValueError:
            return ["Unrecognized response"]

        displays = [
            "yes" if values[0] else "no",
            "yes" if values[1] else "no",
            f"{values[2]} ({FAULTS.get(values[2], 'unknown')})",
            f"{values[3] / 1000.0:g} Nm",
            str(values[4]),
            str(values[5]),
            str(values[6]),
            f"{values[7]} ms",
            f"{values[8]} ms",
            str(values[9]),
            "yes" if values[10] else "no",
            str(values[11]),
            f"{values[12] / 1000.0:g} rad",
            f"{values[13] / 1000.0:g} rad/s",
            f"{values[14] / 1000.0:g} Nm",
            f"{values[15]} C",
            str(values[16]),
            f"0x{values[17]:08x}",
            str(values[18]),
            str(values[19]),
        ]
        return ["Status:"] + [
            f"  {name}: {value}" for name, value in zip(STATUS_FIELDS, displays)
        ]
    if line.startswith("READY "):
        return [line]
    return ["Unrecognized response"]


class Client:
    def __init__(self, port):
        self.port = port
        self.io_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.stop = threading.Event()
        self.maintain_arm = False
        self.torque_command = None

    def exchange_unlocked(self, command):
        self.port.write((command + "\n").encode("ascii"))
        return self.port.readline().decode("ascii", errors="replace")

    def user_exchange(self, command):
        with self.io_lock:
            response = self.exchange_unlocked(command)
            self.record(command.rstrip(), response)
            return response

    def keepalive(self):
        while not self.stop.wait(0.05):
            with self.io_lock:
                with self.state_lock:
                    maintain_arm = self.maintain_arm
                    torque_command = self.torque_command
                if maintain_arm:
                    self.exchange_unlocked("a")
                    if torque_command is not None:
                        self.exchange_unlocked(torque_command)

    def record(self, command, response):
        with self.state_lock:
            if command == "a" and response.strip() == "k,a":
                self.maintain_arm = True
            elif command == "d" and response.strip() == "k,d":
                self.maintain_arm = False
                self.torque_command = None
            elif command.startswith("t,") and response.strip() == "k,t":
                self.torque_command = command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0")
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.25) as port:
        client = Client(port)
        worker = threading.Thread(target=client.keepalive, daemon=True)
        worker.start()
        print("Commands: a, d, t,<Nm>, s, f. Press Ctrl-D or Ctrl-C to exit.")
        try:
            while True:
                command = input("> ")
                if not command:
                    continue
                print(f"TX: {(command + chr(10))!r}")
                response = client.user_exchange(command)
                print(f"RX: {response!r}")
                for message in translate(response):
                    print(message)
        except (EOFError, KeyboardInterrupt):
            print()
        finally:
            client.stop.set()
            worker.join()


if __name__ == "__main__":
    main()
