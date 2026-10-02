#!/usr/bin/env python3
"""Launch the rail teleop node against a pseudo-terminal and check its output."""

from __future__ import annotations

import os
import pty
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


TOPIC = "rail_velocity"
LINES = [b"sp 1500\n", b"sp -250.5\n", b"sp 0.0\n"]
EXPECTED = [1.5, -0.2505, 0.0]
TIMEOUT_S = 10.0


def main() -> int:
    master, slave = pty.openpty()
    node_process = subprocess.Popen(
        [
            "ros2", "run", "rail_interface", "rail-teleop",
            "--ros-args", "-p", f"serial.port:={os.ttyname(slave)}",
        ]
    )
    os.close(slave)

    rclpy.init()
    node = rclpy.create_node("teleop_script_listener")
    received: list[float] = []
    node.create_subscription(
        Float64, TOPIC, lambda message: received.append(message.data),
        qos_profile_sensor_data,
    )

    try:
        deadline = time.monotonic() + TIMEOUT_S
        while node.count_publishers(TOPIC) == 0:
            if node_process.poll() is not None:
                print(f"node exited early with code {node_process.returncode}")
                return 1
            if time.monotonic() > deadline:
                print(f"no publisher on {TOPIC} after {TIMEOUT_S:.0f} s")
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        for line in LINES:
            os.write(master, line)

        while len(received) < len(EXPECTED) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node_process.terminate()
        node_process.wait(timeout=5.0)
        os.close(master)

    print(f"sent:     {[line.decode().strip() for line in LINES]}")
    print(f"expected: {EXPECTED}")
    print(f"received: {received}")

    if len(received) != len(EXPECTED):
        print("FAIL: wrong number of messages")
        return 1
    if any(abs(a - b) > 1e-9 for a, b in zip(received, EXPECTED)):
        print("FAIL: unexpected values")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
