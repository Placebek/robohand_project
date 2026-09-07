#!/usr/bin/env python3
"""Здорова ли камера прямо сейчас. Быстрая проверка на мёртвую область.

HP60C после перезапуска драйвера поверх не отпущенного устройства поднимается
с областью, которая приходит нулями ВСЕГДА — до 17 % рабочего круга, без единой
ошибки в лог.

ВАЖНО: по одному прогону мёртвую область НЕ ОТЛИЧИТЬ от тёмного предмета в
кадре. Чёрное не отражает ИК и даёт такие же стабильные нули. Различает их
только поведение: мёртвая область исчезает после перезапуска драйвера, предмет
остаётся. Поэтому доля печатается машиночитаемой строкой PCT=..., и preflight
сравнивает её до и после перезапуска.

Коды возврата: 0 здорова, 1 мёртвая область, 2 кадров нет.

Запуск:
  python3 tools/_lib/camera_health.py [порог_в_процентах]
"""
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

import numpy as np                 # noqa: E402
import rclpy                       # noqa: E402
from cv_bridge import CvBridge     # noqa: E402
from rclpy.node import Node        # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from suction.config import DEPTH_TOPIC, WORK_CENTRE_PX, WORK_RADIUS_PX  # noqa: E402

THRESHOLD_PCT = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
FRAMES = 12


def main():
    rclpy.init()
    node = Node("camera_health")
    bridge = CvBridge()
    frames = []
    node.create_subscription(
        Image, DEPTH_TOPIC,
        lambda m: frames.append(bridge.imgmsg_to_cv2(m, "passthrough")), 10)

    end = time.monotonic() + 12.0
    while time.monotonic() < end and len(frames) < FRAMES:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if len(frames) < 4:
        print("PCT=-1")
        print(f"  кадров глубины нет ({len(frames)} за 12 с)")
        return 2

    stack = np.stack([f.astype(np.float32) for f in frames])
    dead = (stack == 0).all(axis=0)
    h, w = dead.shape
    cx, cy = WORK_CENTRE_PX
    yy, xx = np.mgrid[0:h, 0:w]
    inside = np.hypot(xx - cx, yy - cy) < WORK_RADIUS_PX
    pct = 100.0 * dead[inside].mean()

    print(f"PCT={pct:.1f}")
    print(f"  кадров {len(frames)}, нулевых пикселей в рабочем круге: {pct:.1f} %")
    if pct <= THRESHOLD_PCT:
        print("  камера здорова")
        return 0
    print(f"  нулей больше порога {THRESHOLD_PCT:g} %: либо мёртвая область")
    print("  камеры, либо тёмный предмет в кадре — по одному прогону не различить")
    return 1


if __name__ == "__main__":
    sys.exit(main())
