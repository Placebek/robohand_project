#!/usr/bin/env python3
"""Дождаться первого кадра глубины. Код возврата 0 — камера отдаёт данные."""
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

import rclpy                    # noqa: E402
from rclpy.node import Node     # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from suction.config import DEPTH_TOPIC  # noqa: E402

timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
rclpy.init()
node = Node("wait_camera")
got = []
node.create_subscription(Image, DEPTH_TOPIC, lambda m: got.append(1), 10)
end = time.monotonic() + timeout
while not got and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if got:
    print("камера отдаёт кадры")
    sys.exit(0)
print(f"за {timeout:g} с кадров глубины не пришло")
sys.exit(1)
