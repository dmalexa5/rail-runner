#!/usr/bin/env python3
"""Plot motor torque, position, and velocity feedback from the UART."""

import argparse
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


SAMPLE_PERIOD_S = 0.01
WINDOW_S = 10.0


def read_feedback(port, stop, samples, lock, errors):
    deadline = time.monotonic()
    while not stop.is_set():
        try:
            port.write(b"f\n")
            fields = port.readline().decode("ascii", errors="replace").strip().split(",")
            if len(fields) == 4 and fields[0] == "f":
                sample = (
                    time.monotonic(),
                    int(fields[1]) / 1000.0,
                    int(fields[2]) / 1000.0,
                    int(fields[3]) / 1000.0,
                )
                with lock:
                    samples.append(sample)
        except (OSError, ValueError, serial.SerialException) as error:
            errors.append(str(error))
            stop.set()
            return

        deadline += SAMPLE_PERIOD_S
        delay = deadline - time.monotonic()
        if delay > 0:
            stop.wait(delay)
        else:
            deadline = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0")
    args = parser.parse_args()

    samples = deque(maxlen=int(WINDOW_S / SAMPLE_PERIOD_S))
    lock = threading.Lock()
    stop = threading.Event()
    errors = []

    with serial.Serial(args.port, 115200, timeout=SAMPLE_PERIOD_S) as port:
        reader = threading.Thread(
            target=read_feedback,
            args=(port, stop, samples, lock, errors),
            daemon=True,
        )
        reader.start()

        figure, axes = plt.subplots(3, 1, sharex=True)
        lines = [axis.plot([], [])[0] for axis in axes]
        axes[0].set_ylabel("Torque (Nm)")
        axes[1].set_ylabel("Position (rad)")
        axes[2].set_ylabel("Velocity (rad/s)")
        axes[2].set_xlabel("Time (s)")

        def update(_frame):
            with lock:
                current = list(samples)
            if not current:
                return lines

            now = current[-1][0]
            times = [sample[0] - now for sample in current]
            values = (
                [sample[3] for sample in current],
                [sample[1] for sample in current],
                [sample[2] for sample in current],
            )
            for axis, line, data in zip(axes, lines, values):
                line.set_data(times, data)
                axis.relim()
                axis.autoscale_view(scalex=False)
            axes[2].set_xlim(-WINDOW_S, 0.0)
            if errors:
                figure.suptitle(errors[0])
            return lines

        animation = FuncAnimation(figure, update, interval=50, cache_frame_data=False)
        try:
            plt.show()
        finally:
            stop.set()
            reader.join()


if __name__ == "__main__":
    main()
