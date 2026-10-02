#!/usr/bin/env python3
"""Print the Nano's serial output until Ctrl+C."""

import argparse
import sys

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0 or /dev/cu.usbserial-*")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, baudrate=115200, timeout=1) as nano:
            while True:
                data = nano.read(nano.in_waiting or 1)
                if data:
                    sys.stdout.write(data.decode("ascii", errors="replace"))
                    sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    except serial.SerialException as error:
        parser.exit(1, f"Serial error: {error}\n")


if __name__ == "__main__":
    main()
