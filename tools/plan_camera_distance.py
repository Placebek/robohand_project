#!/usr/bin/env python3
"""На какой высоте вешать камеру.

Два требования тянут в разные стороны:
  ближе -> точнее глубина (шум растёт примерно как z^2) и крупнее пятно чашки
           в пикселях, то есть надёжнее оценка нормали;
  дальше -> шире поле зрения, то есть больше рабочая зона.

Скрипт считает обе величины для набора дистанций по РЕАЛЬНЫМ интринсикам
камеры и показывает, где границы. Если драйвер камеры запущен, интринсики
берутся из camera_info; иначе задай --fx вручную.

Запуск:
  tools/run_camera_plan.sh            # с живой камерой
  python3 tools/plan_camera_distance.py --fx 500   # прикидка без камеры
"""
import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from suction.config import DATA_CALIB, DEPTH_MIN_M  # noqa: E402

CUP_MM = 35.0            # чашка ES01
MIN_PATCH_PX = 20        # меньше — на подгонку плоскости не хватает точек
GOOD_PATCH_PX = 35
MIN_MARKER_PX = 20       # порог надёжной детекции ArUco
GOOD_MARKER_PX = 30
IMG_W, IMG_H = 640, 480


def get_fx_from_ros(timeout_s=8.0):
    """Интринсики из живого camera_info. None, если драйвер не запущен."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo
        from suction.config import DEPTH_INFO_TOPIC
    except ImportError:
        return None

    import time
    rclpy.init()
    node = Node("camera_plan")
    box = {}
    node.create_subscription(CameraInfo, DEPTH_INFO_TOPIC,
                             lambda m: box.setdefault("k", m.k), 10)
    deadline = time.monotonic() + timeout_s
    while "k" not in box and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return float(box["k"][0]) if "k" in box else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fx", type=float, default=None,
                    help="фокус в пикселях. Без него берётся из camera_info")
    args = ap.parse_args()

    fx = args.fx
    source = "аргумент --fx"
    if fx is None:
        fx = get_fx_from_ros()
        source = "camera_info (живая камера)"
    if fx is None:
        print("Камера не отвечает и --fx не задан.")
        print("Запусти драйвер (tools/start_camera.sh) или укажи --fx вручную.")
        return 1

    # размер маркера на распечатанной доске, с учётом реального масштаба печати
    spec_path = DATA_CALIB / "charuco_board.yaml"
    marker_mm = None
    if spec_path.exists():
        spec = yaml.safe_load(spec_path.read_text())
        nominal_sq = spec["square_length_m"] * 1000.0
        measured_sq = spec.get("measured_square_mm") or nominal_sq
        marker_mm = spec["marker_length_m"] * 1000.0 * (measured_sq / nominal_sq)

    print(f"fx = {fx:.1f} px   (источник: {source})")
    hfov = 2 * __import__("math").degrees(__import__("math").atan(IMG_W / 2 / fx))
    print(f"горизонтальный угол обзора ≈ {hfov:.0f}°")
    if marker_mm:
        print(f"маркер на распечатке: {marker_mm:.2f} мм")
    print()

    head = f"{'z, мм':>7} {'пятно чашки':>12} {'маркер':>8} {'ширина зоны':>12}  вердикт"
    print(head)
    print("-" * len(head))

    for z in (200, 250, 300, 350, 400, 450, 500, 600, 700, 800, 1000):
        patch_px = CUP_MM * fx / z
        marker_px = marker_mm * fx / z if marker_mm else 0.0
        fov_mm = IMG_W * z / fx

        notes = []
        if z < DEPTH_MIN_M * 1000:
            notes.append("НИЖЕ МИНИМУМА камеры")
        elif z < DEPTH_MIN_M * 1000 * 1.25:
            notes.append("впритык к минимуму")
        if patch_px < MIN_PATCH_PX:
            notes.append("пятно мелкое")
        elif patch_px < GOOD_PATCH_PX:
            notes.append("пятно на грани")
        if marker_mm and marker_px < MIN_MARKER_PX:
            notes.append("маркеры не детектятся")
        elif marker_mm and marker_px < GOOD_MARKER_PX:
            notes.append("маркеры на грани")

        verdict = "; ".join(notes) if notes else "годится"
        print(f"{z:>7} {patch_px:>10.0f} px {marker_px:>6.0f} px "
              f"{fov_mm:>9.0f} мм  {verdict}")

    print()
    print("Как читать:")
    print(f"  пятно чашки — сколько пикселей занимает круг {CUP_MM:g} мм. По нему")
    print(f"                подгоняется плоскость; меньше {MIN_PATCH_PX} px точек не хватит.")
    print("  маркер      — сторона маркера ArUco при hand-eye калибровке.")
    print("  ширина зоны — сколько миллиметров сцены влезает в кадр по горизонтали.")
    print()
    print("Ставь камеру на минимальную дистанцию, при которой рабочая зона ещё")
    print("покрывается: шум глубины растёт примерно как квадрат расстояния, и")
    print("каждый лишний сантиметр высоты оплачивается точностью нормали.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
