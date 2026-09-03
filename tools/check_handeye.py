#!/usr/bin/env python3
"""Проверка калибровки от пикселя до координат руки. Ничего не двигает.

Невязка решения AX=XB ничего не говорит о том, правильно ли пиксель
превращается в точку в базе робота: перепутанный знак или подменённая система
координат дают прекрасный остаток и полностью неверный результат. Здесь
цепочка проверяется целиком и по частям.

Что считается:

1. СОВМЕЩЕНИЕ RGB И ГЛУБИНЫ. Берутся углы ChArUco. Для каждого угла есть две
   независимые оценки его положения в системе камеры: одна из позы доски по
   RGB, вторая из карты глубины в том же пикселе. Если потоки не совмещены
   попиксельно, оценки разойдутся, и вся дальнейшая арифметика поедет.

2. ПЛОСКОСТЬ ПО ГЛУБИНЕ. Углы, восстановленные из карты глубины, обязаны
   лечь в одну плоскость. Это настоящее измерение — в отличие от углов из
   модели доски, которые плоские по построению и ничего не проверяют.

3. СКВОЗНАЯ ПРОВЕРКА ПОВОРОТА. Если доска сейчас на фланце, её нормаль,
   посчитанная через камеру и калибровку, обязана совпасть с осью инструмента,
   которую независимо сообщают энкодеры руки. Расхождение — это ошибка
   поворотной части калибровки, измеренная двумя разными приборами.

4. ПЕРЕВОД В БАЗУ. Углы доски переводятся матрицей из handeye.json в
   координаты робота. Доска лежит на столе, значит все углы должны получить
   ОДНУ И ТУ ЖЕ высоту z. Разброс z по углам — ошибка калибровки, выраженная
   в миллиметрах на реальной сцене, а не в абстрактной невязке.

Доску для проверки положить НА СТОЛ, лицом вверх, в рабочий круг.

Запуск:
  tools/run_check_handeye.sh
"""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cv2                      # noqa: E402
import numpy as np              # noqa: E402
import rclpy                    # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node     # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

from suction.config import (DATA_CALIB, DEPTH_TOPIC, RGB_TOPIC,  # noqa: E402
                            RGB_INFO_TOPIC)
from tools.capture_handeye import detect, load_board  # noqa: E402
from tools.solve_handeye import euler_to_R  # noqa: E402


class Both(Node):
    def __init__(self):
        super().__init__("handeye_check")
        self.b = CvBridge()
        self.rgb = None
        self.depth = []
        self.K = None
        self.D = None
        self.create_subscription(Image, RGB_TOPIC,
                                 lambda m: setattr(self, "rgb",
                                                   self.b.imgmsg_to_cv2(m, "bgr8")), 10)
        self.create_subscription(Image, DEPTH_TOPIC,
                                 lambda m: self.depth.append(
                                     self.b.imgmsg_to_cv2(m, "passthrough")), 10)
        self.create_subscription(CameraInfo, RGB_INFO_TOPIC, self._info, 10)

    def _info(self, m):
        self.K = np.array(m.k).reshape(3, 3)
        self.D = np.array(m.d).reshape(1, -1)

    def collect(self, seconds=4.0, frames=30):
        end = time.monotonic() + seconds
        while time.monotonic() < end and len(self.depth) < frames:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.rgb is None or not self.depth or self.K is None:
            raise RuntimeError("нет кадров или интринсик — запущен ли драйвер?")
        st = np.stack([d.astype(np.float32) for d in self.depth])
        st[st == 0] = np.nan
        return self.rgb.copy(), np.nanmedian(st, axis=0)


def main():
    path = DATA_CALIB / "handeye.json"
    if not path.exists():
        print(f"нет {path} — сначала tools/run_solve_handeye.sh")
        return 1
    cal = json.loads(path.read_text())
    Tcb = np.array(cal["T_камера_в_базу"])
    print(f"калибровка: набор {cal['набор']}, метод {cal['метод']}, "
          f"разброс {cal['разброс_фланец_доска_мм']:.2f} мм / "
          f"{cal['разброс_фланец_доска_град']:.2f}°")

    board, dictionary = load_board()   # load_board отдаёт пару
    rclpy.init()
    node = Both()
    try:
        rgb, depth = node.collect()
        K, D = node.K, node.D
        corners, ids = detect(rgb, board, dictionary)
        n = 0 if ids is None else len(ids)
        print(f"\nуглов доски найдено: {n}")
        if n < 8:
            print("Мало. Положи доску на стол лицом вверх в рабочий круг.")
            return 1

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            corners, ids, board, K, D, None, None)
        if not ok:
            print("поза доски не оценилась")
            return 1
        R = cv2.Rodrigues(rvec)[0]

        obj = np.array(board.chessboardCorners, dtype=np.float64)
        pix = corners.reshape(-1, 2)
        idx = ids.reshape(-1)

        # --- 1. RGB против глубины ------------------------------------
        p_rgb, p_depth, used = [], [], []
        for k, (u, v) in zip(idx, pix):
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= vi < depth.shape[0] and 0 <= ui < depth.shape[1]):
                continue
            z = depth[vi, ui]
            if not np.isfinite(z) or z <= 0:
                continue
            zm = float(z) / 1000.0
            p_depth.append([(ui - K[0, 2]) * zm / K[0, 0],
                            (vi - K[1, 2]) * zm / K[1, 1], zm])
            p_rgb.append(R @ obj[k] + tvec.reshape(3))
            used.append(k)
        p_rgb = np.array(p_rgb)
        p_depth = np.array(p_depth)
        print(f"углов с валидной глубиной: {len(used)} из {n}")
        if len(used) < 6:
            print("Глубина в углах доски дырявая — чёрные квадраты ИК не "
                  "отражают. Это ожидаемо, но для проверки нужно 6+.")
        else:
            d = np.linalg.norm(p_rgb - p_depth, axis=1) * 1000.0
            print(f"\n1. Совмещение RGB и глубины (расхождение двух оценок):")
            print(f"   median {np.median(d):.1f} мм, p95 {np.percentile(d,95):.1f}, "
                  f"max {d.max():.1f}")
            print("   " + ("потоки совмещены" if np.median(d) < 8 else
                           "ПОТОКИ НЕ СОВМЕЩЕНЫ — глубину нельзя брать "
                           "по пикселю из RGB"))

        # --- 2. плоскость по измеренной глубине -----------------------
        if len(used) >= 6:
            c = p_depth.mean(axis=0)
            _, _, vt = np.linalg.svd(p_depth - c)
            res = (p_depth - c) @ vt[2]
            print(f"\n2. Плоскость углов по ГЛУБИНЕ: остаток median "
                  f"{np.median(np.abs(res))*1000:.2f} мм, "
                  f"max {np.abs(res).max()*1000:.2f}")

        # --- 3. нормаль доски против оси инструмента -------------------
        normal_cam = R @ np.array([0.0, 0.0, 1.0])
        normal_base = Tcb[:3, :3] @ normal_cam
        try:
            from suction.arm.client import ArmClient
            with ArmClient() as arm:
                flange = arm.pose()
                on_cup = arm.gripper_state()
        except Exception as e:
            flange, on_cup = None, False
            print(f"\n3. Сквозную проверку поворота пропускаю: рука не "
                  f"отвечает ({type(e).__name__})")

        if flange is not None:
            axis = euler_to_R(flange[3:], ("z", "y", "x"), True) @ np.array(
                [0.0, 0.0, 1.0])
            ang = np.degrees(np.arccos(abs(np.clip(np.dot(normal_base, axis),
                                                   -1, 1))))
            print(f"\n3. Нормаль доски против оси инструмента:")
            print(f"   через камеру и калибровку: "
                  f"[{normal_base[0]:+.4f} {normal_base[1]:+.4f} {normal_base[2]:+.4f}]")
            print(f"   по энкодерам руки:         "
                  f"[{axis[0]:+.4f} {axis[1]:+.4f} {axis[2]:+.4f}]")
            print(f"   расхождение: {ang:.2f}°")
            if not on_cup:
                print("   (присоска выключена — если доска НЕ на фланце,")
                print("    это число ничего не значит)")
            elif ang < 3:
                print("   -> поворотная часть калибровки в порядке")
            else:
                print("   -> великовато; часть от податливости крепления")

        # --- 4. перевод в базу ----------------------------------------
        base = (Tcb[:3, :3] @ p_rgb.T).T + Tcb[:3, 3]
        z = base[:, 2] * 1000.0

        # Лежит ли доска плашмя. Если нет, разброс z меряет её наклон, а не
        # ошибку калибровки, и вывод про «что-то в цепочке неверно» будет ложью.
        tilt = np.degrees(np.arccos(abs(np.clip(normal_base[2], -1, 1))))
        flat = tilt < 10.0
        print(f"\n4. Углы доски в координатах базы робота:")
        print(f"   x {base[:,0].min()*1000:+7.1f} .. {base[:,0].max()*1000:+7.1f} мм")
        print(f"   y {base[:,1].min()*1000:+7.1f} .. {base[:,1].max()*1000:+7.1f} мм")
        print(f"   z {z.min():+7.1f} .. {z.max():+7.1f} мм   "
              f"(разброс {z.max()-z.min():.1f})")
        print(f"\n   наклон плоскости доски к горизонту базы: {tilt:.1f}°")
        if not flat:
            print(f"\n   ДОСКА НЕ ЛЕЖИТ ПЛАШМЯ — она накренена на {tilt:.0f}°.")
            print("   Разброс z меряет этот наклон, а не ошибку калибровки.")
            print("   Сними доску с присоски (tools/run_gripper.sh off,")
            print("   придерживая рукой), положи на стол лицом вверх в")
            print("   рабочий круг и запусти проверку заново.")
        else:
            print(f"\n   Доска лежит плашмя, значит z у всех углов должен")
            print(f"   совпадать. Разброс {z.max()-z.min():.1f} мм — это ошибка")
            print(f"   калибровки на реальной сцене, а не абстрактная невязка.")
            if z.max() - z.min() < 5:
                print("   -> для чашки 35 мм этого достаточно")
            elif z.max() - z.min() < 15:
                print("   -> грубо, но чашка попадёт; для RQ2 мало")
            else:
                print("   -> ПЛОХО, что-то в цепочке неверно")

        centre = base.mean(axis=0) * 1000.0
        print(f"\n   центр доски в базе: x={centre[0]:+.1f} y={centre[1]:+.1f} "
              f"z={centre[2]:+.1f} мм")
        print("   Сверь глазами: рука должна дотягиваться до этой точки.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
