#!/usr/bin/env python3
"""Проверка пригодности глубины HP60C для оценки нормалей (план §12.1, риск §9).

Это гейт для RQ2: если нормаль на масштабе присоски (35 мм у ES01) шумит сильнее
~5°, вся ветка «выравнивание по нормали» под вопросом — и узнать это надо
до того, как вложишься в сбор датасета.

Протокол:
  1. Положи плоский жёсткий лист (не бумагу — фанеру/пластик) под камеру.
  2. Мышью выдели на нём область → замер по N кадрам (пробел).
  3. Наклони лист на известный угол (телефон с инклинометром рядом) → ещё замер.
  4. Скрипт сравнит разницу углов между замерами с тем, что ты ввёл руками.

Абсолютный угол к оптической оси не проверяем — он требует калибровки
установки камеры. Разница углов между двумя позами листа от неё не зависит,
а для RQ2 важна именно она.

Запуск: tools/run_depth_check.sh   (в другом терминале — tools/start_camera.sh)
"""
import json
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from suction.config import (DATA_CALIB, DEPTH_INFO_TOPIC, DEPTH_MIN_M,  # noqa: E402
                            DEPTH_TOPIC)

FRAMES_PER_MEASUREMENT = 30
# Заводская чашка ES01: наружный диаметр 35 мм (замер по факту). Это масштаб,
# на котором надо оценивать нормаль — плоскость должна быть плоской на всём
# пятне контакта, а не в одной точке. Чашка почти вдвое больше типовой 15-20 мм,
# и это ужесточает требование к плоскостности, а не смягчает его.
CUP_DIAMETER_MM = 35.0


class DepthProbe(Node):
    def __init__(self):
        super().__init__("depth_probe")
        self.bridge = CvBridge()
        self.depth_mm = None
        self.K = None
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._info_cb, 10)

    def _depth_cb(self, msg):
        self.depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)


def backproject(depth_mm, K, roi):
    """ROI карты глубины -> облако точек (N,3) в мм, координаты камеры."""
    x0, y0, w, h = roi
    patch = depth_mm[y0:y0 + h, x0:x0 + w].astype(np.float64)
    vs, us = np.mgrid[y0:y0 + h, x0:x0 + w]
    valid = patch > 0
    z = patch[valid]
    u = us[valid].astype(np.float64)
    v = vs[valid].astype(np.float64)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=1)


def fit_plane(points):
    """МНК-подгонка плоскости. Возвращает (нормаль, RMS-невязка в мм)."""
    if len(points) < 10:
        return None, None
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[2]
    if normal[2] > 0:          # ориентируем на камеру
        normal = -normal
    residual = centered @ normal
    return normal, float(np.sqrt((residual ** 2).mean()))


PATCH_VALID_FRACTION = 0.9   # пятачок должен почти целиком лежать на поверхности
PATCH_DEPTH_BAND_MM = 60.0   # разброс глубины внутри пятачка сверх этого — значит зацепили фон


def cup_scale_normals(K, roi, depth_mm):
    """Нормали на пятачках размером с чашку — то, что реально увидит политика.

    Разброс между такими пятачками и есть практический шум оценки нормали.

    Пятачки с дырами и со ступенькой на фон отбрасываются: подгонка плоскости
    по точкам, часть которых лежит на столе в полуметре позади, даёт нормаль
    в десятки градусов и отравляет всю статистику. Отбрасывать надо ДО расчёта,
    а не убирать выбросы после — иначе непонятно, что именно ты убрал.

    Возвращает список (нормаль, RMS, (cx, cy)) — координаты нужны, чтобы
    отделить центр кадра от периферии: у дешёвых RGB-D карта глубины к краям
    систематически выгнута, и это видно только в таком разрезе.
    """
    x0, y0, w, h = roi
    patch = depth_mm[y0:y0 + h, x0:x0 + w]
    valid = patch[patch > 0]
    if valid.size == 0:
        return []
    z_med = float(np.median(valid))
    # радиус чашки в пикселях на текущей дистанции
    r_px = max(3, int(round(CUP_DIAMETER_MM / 2 * K[0, 0] / z_med)))

    out = []
    step = r_px
    for cy in range(y0 + r_px, y0 + h - r_px, step):
        for cx in range(x0 + r_px, x0 + w - r_px, step):
            sub_d = depth_mm[cy - r_px:cy + r_px, cx - r_px:cx + r_px]
            m = sub_d > 0
            if m.mean() < PATCH_VALID_FRACTION:
                continue
            zz = sub_d[m]
            if float(zz.max() - zz.min()) > PATCH_DEPTH_BAND_MM:
                continue
            pts = backproject(depth_mm, K, (cx - r_px, cy - r_px, 2 * r_px, 2 * r_px))
            n, rms = fit_plane(pts)
            if n is not None:
                out.append((n, rms, (cx, cy)))
    return out


def spread_stats(cups, K):
    """Разброс нормалей, отдельно по центру кадра и по периферии."""
    normals = [c[0] for c in cups]
    mean_n = np.mean(normals, axis=0)
    mean_n /= np.linalg.norm(mean_n)
    sp = np.array([angle_between(n, mean_n) for n in normals])
    cx = np.array([c[2][0] for c in cups], dtype=float)
    cy = np.array([c[2][1] for c in cups], dtype=float)
    # «центр» — внутри половины кадра вокруг главной точки
    central = (np.abs(cx - K[0, 2]) < 160) & (np.abs(cy - K[1, 2]) < 120)
    return {
        "median_deg": float(np.median(sp)),
        "p95_deg": float(np.percentile(sp, 95)),
        "n_patches": len(cups),
        "center_median_deg": float(np.median(sp[central])) if central.sum() >= 5 else None,
        "edge_median_deg": float(np.median(sp[~central])) if (~central).sum() >= 5 else None,
    }


def angle_between(n1, n2):
    c = float(np.clip(np.dot(n1, n2) / (np.linalg.norm(n1) * np.linalg.norm(n2)), -1, 1))
    return float(np.degrees(np.arccos(c)))


def measure(node, roi, frames=FRAMES_PER_MEASUREMENT):
    """Замер по нескольким кадрам: даёт и среднюю нормаль, и её временной дрейф."""
    plane_normals, plane_rms, cup_spreads, distances = [], [], [], []

    collected = 0
    deadline = time.monotonic() + 10.0
    while collected < frames and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.depth_mm is None:
            continue
        depth = node.depth_mm.copy()
        pts = backproject(depth, node.K, roi)
        n, rms = fit_plane(pts)
        if n is None:
            continue
        plane_normals.append(n)
        plane_rms.append(rms)
        distances.append(float(np.median(pts[:, 2])))

        cups = cup_scale_normals(node.K, roi, depth)
        if len(cups) >= 5:
            cup_spreads.append(spread_stats(cups, node.K))
        collected += 1

    if not plane_normals:
        return None

    mean_normal = np.mean(plane_normals, axis=0)
    mean_normal /= np.linalg.norm(mean_normal)
    temporal_jitter = [angle_between(n, mean_normal) for n in plane_normals]

    return {
        "roi": list(roi),
        "frames": collected,
        "distance_mm": float(np.median(distances)),
        "normal": mean_normal.tolist(),
        "planarity_rms_mm": float(np.mean(plane_rms)),
        "temporal_jitter_deg": float(np.std(temporal_jitter)),
        "temporal_jitter_max_deg": float(np.max(temporal_jitter)),
        "cup_scale": {
            k: (float(np.median([c[k] for c in cup_spreads if c[k] is not None]))
                if any(c[k] is not None for c in cup_spreads) else None)
            for k in ("median_deg", "p95_deg", "center_median_deg", "edge_median_deg")
        } if cup_spreads else None,
        "cup_patches": int(np.median([c["n_patches"] for c in cup_spreads])) if cup_spreads else 0,
    }


def main():
    rclpy.init()
    node = DepthProbe()

    print("Жду кадры глубины и camera_info... (камера: tools/start_camera.sh)")
    deadline = time.monotonic() + 20
    while (node.depth_mm is None or node.K is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.depth_mm is None:
        print("Кадры глубины не приходят вообще. Драйвер камеры запущен?")
        print("  проверь:  ros2 topic hz " + DEPTH_TOPIC)
        return

    # Отдельный случай, который легко принять за отсутствие камеры: кадры идут,
    # но целиком нулевые. У HP60C так выглядит сцена ближе паспортных 0.2 м —
    # дальномер просто не отдаёт значений, а не сообщает об ошибке.
    if not (node.depth_mm > 0).any():
        print()
        print("Кадры приходят, но глубина ПУСТАЯ: ноль валидных пикселей.")
        print(f"Самая частая причина — сцена ближе минимума {DEPTH_MIN_M:.2f} м.")
        print("Отодвинь камеру на 0.30-0.45 м и запусти снова.")
        print("Проверить дистанцию, не гадая: tools/run_camera_plan.sh")
        print()
        print("Если отодвинул, а глубина всё равно пустая — смотри на сцену:")
        print("зеркало, стекло и глубокий чёрный дальномер тоже не видит.")
        return
    if node.K is None:
        print(f"Нет camera_info на {DEPTH_INFO_TOPIC}. Проверь имя топика:\n"
              f"  ros2 topic list | grep camera_info")
        return

    print(f"Интринсики: fx={node.K[0,0]:.1f} fy={node.K[1,1]:.1f} "
          f"cx={node.K[0,2]:.1f} cy={node.K[1,2]:.1f}")

    roi = None
    results = []
    window = "Depth (SPACE - замер, R - выделить ROI, Q - выход)"
    cv2.namedWindow(window)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.depth_mm is None:
                continue
            depth = node.depth_mm
            vis = np.clip(depth, 0, 1200).astype(np.float32) / 1200.0 * 255
            vis = cv2.applyColorMap(vis.astype(np.uint8), cv2.COLORMAP_JET)
            vis[depth == 0] = (0, 0, 0)
            if roi is not None:
                x0, y0, w, h = roi
                cv2.rectangle(vis, (x0, y0), (x0 + w, y0 + h), (255, 255, 255), 2)
            cv2.putText(vis, f"замеров: {len(results)}", (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow(window, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r") or (key == ord(" ") and roi is None):
                sel = cv2.selectROI(window, vis, showCrosshair=True)
                roi = sel if sel[2] > 10 and sel[3] > 10 else None
                if roi is None:
                    print("ROI слишком мал.")
            elif key == ord(" ") and roi is not None:
                print(f"\nЗамер по {FRAMES_PER_MEASUREMENT} кадрам...")
                r = measure(node, roi)
                if r is None:
                    print("Не набралось валидных точек — сдвинь ROI на область без дыр.")
                    continue
                tilt = input("Угол наклона листа сейчас (градусы, Enter если не мерил): ").strip()
                r["reported_tilt_deg"] = float(tilt) if tilt else None
                results.append(r)

                print(f"  дистанция:                {r['distance_mm']:.0f} мм")
                print(f"  RMS невязки плоскости:    {r['planarity_rms_mm']:.2f} мм  "
                      f"(шум глубины на масштабе ROI)")
                print(f"  дрожание нормали во вр.:  {r['temporal_jitter_deg']:.2f}° "
                      f"(max {r['temporal_jitter_max_deg']:.2f}°)")
                cs = r["cup_scale"]
                if cs is not None:
                    v = cs["median_deg"]
                    verdict = "ОК для RQ2" if v < 5 else "ПЛОХО — RQ2 под вопросом"
                    print(f"  разброс нормали на чашке: {v:.2f}° "
                          f"(p95 {cs['p95_deg']:.2f}°, пятачков {r['cup_patches']})"
                          f"  <-- {verdict}")
                    c, e = cs["center_median_deg"], cs["edge_median_deg"]
                    if c is not None and e is not None:
                        print(f"      центр кадра {c:.2f}°   периферия {e:.2f}°", end="")
                        print("   <-- глубина выгнута к краям" if e > 2 * c else "")
                else:
                    print("  пятачков не набралось: ROI мелкий или в дырах")

                if len(results) >= 2:
                    prev = results[-2]
                    d = angle_between(np.array(prev["normal"]), np.array(r["normal"]))
                    print(f"  угол к предыдущему замеру: {d:.2f}°", end="")
                    if prev["reported_tilt_deg"] is not None and r["reported_tilt_deg"] is not None:
                        expected = abs(r["reported_tilt_deg"] - prev["reported_tilt_deg"])
                        print(f"  (заявлено {expected:.1f}°, ошибка {abs(d - expected):.2f}°)")
                    else:
                        print()
    finally:
        if results:
            DATA_CALIB.mkdir(parents=True, exist_ok=True)
            out = DATA_CALIB / f"depth_check_{datetime.now():%Y%m%d_%H%M%S}.json"
            out.write_text(json.dumps({"intrinsics": node.K.tolist(),
                                       "cup_diameter_mm": CUP_DIAMETER_MM,
                                       "measurements": results}, indent=2))
            print(f"\nСохранено: {out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
