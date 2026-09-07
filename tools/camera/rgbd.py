import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

# Топики берутся из config, а не дублируются здесь: единая точка правды.
from suction.config import DEPTH_TOPIC, RGB_TOPIC  # noqa: E402

click_point = [320, 240]


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        click_point[0] = x
        click_point[1] = y


class RgbdViewer(Node):
    def __init__(self):
        super().__init__("rgbd_viewer")
        self.bridge = CvBridge()
        self.rgb = None
        self.depth_mm = None
        self.create_subscription(Image, RGB_TOPIC, self.rgb_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_cb, 10)

    def rgb_cb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_cb(self, msg):
        self.depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


def main():
    rclpy.init()
    node = RgbdViewer()

    cv2.namedWindow("RGB")
    cv2.namedWindow("Depth - klik dlya zamera")
    mouse_bound = False
    depth_shown_count = 0

    print("Ожидание кадров от ascamera_node "
          "(запусти в другом терминале: ros2 launch ascamera hp60c.launch.py)...")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.rgb is not None:
                cv2.imshow("RGB", node.rgb)

            if node.depth_mm is not None:
                depth = node.depth_mm
                h, w = depth.shape[:2]
                px = min(click_point[0], w - 1)
                py = min(click_point[1], h - 1)
                point_mm = int(depth[py, px])

                valid = depth[depth > 0]
                min_val = int(valid.min()) if valid.size else 0
                max_val = int(valid.max()) if valid.size else 0
                mean_val = float(valid.mean()) if valid.size else 0.0

                depth_vis = np.clip(depth, 0, 4000).astype(np.float32) / 4000.0 * 255
                depth_vis = depth_vis.astype(np.uint8)
                depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                depth_color[depth == 0] = (0, 0, 0)

                cv2.drawMarker(depth_color, (px, py), (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
                cv2.putText(depth_color, f"{point_mm} mm", (px + 8, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(depth_color,
                            f"min={min_val} max={max_val} mean={mean_val:.0f} mm",
                            (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                cv2.imshow("Depth - klik dlya zamera", depth_color)
                print(f"\rpoint({px},{py})={point_mm} mm   "
                      f"min={min_val} max={max_val} mean={mean_val:.0f}   ", end="")
                depth_shown_count += 1

            key = cv2.waitKey(1) & 0xFF

            # Имя окна ТОЛЬКО латиницей: Qt-бэкенд не находит окна с кириллицей,
            # и привязка мыши падает молча — клик по глубине не работал вовсе.
            # окно реально создаётся в Qt-бэкенде не сразу после первого imshow
            # (момент непредсказуем при совместной работе с rclpy), поэтому
            # просто повторяем попытку привязки, пока Qt не будет готов
            if depth_shown_count >= 1 and not mouse_bound:
                try:
                    cv2.setMouseCallback("Depth - klik dlya zamera", on_mouse)
                    mouse_bound = True
                except cv2.error:
                    pass

            if key == ord("q"):
                break
            elif key == ord("s") and node.rgb is not None and node.depth_mm is not None:
                cv2.imwrite("rgb.png", node.rgb)
                cv2.imwrite("depth.png", node.depth_mm)
                print("\nСохранено: rgb.png, depth.png (глубина в мм, 16 бит)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
