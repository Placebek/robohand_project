#!/usr/bin/env python3
"""Какая часть рабочего круга камеры достижима рукой. Ничего не двигает.

Место для камеры выбирают по качеству глубины, чистоте круга и запасу до края
стола — и дважды подряд забыли проверить главное: дотягивается ли туда рука.
Первый раз это стоило переноса камеры и пересъёмки hand-eye, второй раз
обнаружилось уже после следующей калибровки: центр круга сел на y=-460 при
пределе досягаемости -455, и половина зоны оказалась декоративной.

Проверка стоит одной команды InverseKin на узел и не двигает руку.
Запускать СРАЗУ после решения hand-eye, до любых измерений: число нормалей,
намеренное по зоне, половину которой рука не достанет, не описывает рабочие
условия.

Круг из handeye.json проецируется на плоскость стола (z=0 в базе), в узлах
сетки спрашивается обратная кинематика для позы «инструмент вниз».

Запуск:
  tools/arm/reach_map.sh
  tools/arm/reach_map.sh --z 200     # чашка на 200 мм над столом
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

from suction.arm.client import ArmClient, run_tool  # noqa: E402
from suction.calib.handeye import load_handeye  # noqa: E402
from suction.config import (TOOL_LENGTH_MM, WORK_CENTRE_PX,  # noqa: E402
                            WORK_RADIUS_PX)


def table_point(u, v, K, R, t):
    """Пиксель -> точка на плоскости стола z=0 в базе, мм."""
    d = R @ (np.linalg.inv(K) @ np.array([u, v, 1.0]))
    if abs(d[2]) < 1e-9:
        return None
    return (t + (0.0 - t[2]) / d[2] * d) * 1000.0


def main():
    ap = argparse.ArgumentParser(
        description="Достижимость рабочего круга камеры. Руку не двигает.")
    ap.add_argument("--z", type=float, default=120.0,
                    help="высота ГУБЫ ЧАШКИ над столом, мм (по умолчанию 120)")
    ap.add_argument("--step", type=float, default=24.0,
                    help="шаг сетки в пикселях")
    args = ap.parse_args()

    try:
        cal, T = load_handeye()
    except FileNotFoundError as e:
        print(e)
        return 1
    print(f"калибровка: набор {cal['набор']}")

    # Интринсики те же, что у остальных инструментов: из camera_info.
    K = np.array([[584.1, 0.0, 320.0], [0.0, 583.1, 274.8], [0.0, 0.0, 1.0]])
    R, t = T[:3, :3], T[:3, 3]
    cx, cy = WORK_CENTRE_PX
    rad = WORK_RADIUS_PX
    print(f"камера в базе: x={t[0]*1000:+.1f} y={t[1]*1000:+.1f} "
          f"z={t[2]*1000:+.1f} мм")

    pts = []
    for du in np.arange(-rad, rad + 1, args.step):
        for dv in np.arange(-rad, rad + 1, args.step):
            if du * du + dv * dv > rad * rad:
                continue
            p = table_point(cx + du, cy + dv, K, R, t)
            if p is not None:
                pts.append(p)
    pts = np.array(pts)
    centre = table_point(cx, cy, K, R, t)
    print(f"центр круга на столе: x={centre[0]:+.0f} y={centre[1]:+.0f} мм")
    print(f"круг: x {pts[:,0].min():+.0f}..{pts[:,0].max():+.0f}  "
          f"y {pts[:,1].min():+.0f}..{pts[:,1].max():+.0f} мм")
    print(f"узлов {len(pts)}, спрашиваю обратную кинематику...")

    z_flange = args.z + TOOL_LENGTH_MM
    with ArmClient() as arm:
        ok = np.array([arm.reachable([p[0], p[1], z_flange, 0.0, 0.0, 0.0])
                       for p in pts])

    print(f"\nДОСТИЖИМО {ok.sum()} из {len(ok)} = {100*ok.mean():.0f} % круга "
          f"при чашке на {args.z:g} мм над столом")
    if ok.any():
        g = pts[ok]
        print(f"  достижимая часть: x {g[:,0].min():+.0f}..{g[:,0].max():+.0f}"
              f"  y {g[:,1].min():+.0f}..{g[:,1].max():+.0f} мм")

    # Карта: строки — y, столбцы — x, шаг 20 мм.
    ys = sorted({round(p[1] / 20) * 20 for p in pts}, reverse=True)
    xs = sorted({round(p[0] / 20) * 20 for p in pts})
    print()
    for yv in ys:
        row = ""
        for xv in xs:
            cell = [o for p, o in zip(pts, ok)
                    if round(p[1] / 20) * 20 == yv
                    and round(p[0] / 20) * 20 == xv]
            row += "." if not cell else ("#" if all(cell) else "x")
        print(f"  y={yv:+5.0f} {row}")
    print(f"         x от {xs[0]:+.0f} до {xs[-1]:+.0f}, шаг 20 мм")
    print("         # достаёт, x не достаёт, . вне круга")

    if ok.mean() < 0.8:
        far = pts[~ok]
        shift = abs(far[:, 1]).max() - 455.0
        print(f"\nЗОНА ОБРЕЗАНА ДОСЯГАЕМОСТЬЮ. Недостающее по y около "
              f"{shift:.0f} мм.")
        print("Либо двигать камеру В СТОРОНУ ОСНОВАНИЯ примерно на столько же")
        print("(и переснимать hand-eye), либо работать ближней половиной круга")
        print("и записать это ограничение в условия измерений.")
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
